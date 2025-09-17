# final/Auto_Cal_Logic.py
import time
import statistics
import numpy as np
import pandas as pd
from tkinter import messagebox
import tkinter as tk
import threading
import os

from Cert_Generator import generate_certificate

def run_calibration(app):
    """
    The main logic for the automated calibration sequence.
    """
    # --- BEGIN NEW VALIDATION LOGIC ---
    for dut in app.active_duts:
        wip = dut.get('wip', '').strip()
        serial = dut.get('serial', '').strip()
        model = dut.get('model', '').strip()
        
        if not all([wip, serial, model]):
            error_message = f"DUT {dut['channel']+1} is missing required information.\n\nPlease provide a value for:\n"
            missing_fields = []
            if not wip: missing_fields.append("- WIP #")
            if not serial: missing_fields.append("- S/N")
            if not model: missing_fields.append("- Model #")
            
            messagebox.showerror("Missing Information", error_message + "\n".join(missing_fields))
            app.is_calibrating = False
            app.start_button.config(state=tk.NORMAL)
            app.e_stop_button.config(state=tk.DISABLED)
            return 
    # --- END NEW VALIDATION LOGIC ---

    run_start_time = time.time()
    try:
        master_setpoints = set()
        for i in range(0, 101, 10):
            master_setpoints.add(round(app.standard_fs_value * i / 100, 2))
        for dut in app.active_duts:
            for i in range(0, 101, 10):
                master_setpoints.add(round(dut['fs'] * i / 100, 2))
        setpoints = sorted(list(master_setpoints))
        app.log_message(f"Generated composite setpoints: {setpoints}")

        app.after(0, app._setup_error_plot, setpoints)

        dut_specific_setpoints = {dut['channel']: {round(dut['fs'] * i / 100, 2) for i in range(0, 101, 10)} for dut in app.active_duts}
        
        app.completed_duts.clear()

        for sp in setpoints:
            if not app.is_calibrating: break
            
            for dut in app.active_duts:
                if sp > dut['fs'] * 1.05 and dut['channel'] not in app.completed_duts:
                    app.log_message(f"--- DUT {dut['channel']+1} ({dut['fs']} Torr) range completed. Hiding trace. ---")
                    app.completed_duts.add(dut['channel'])

            if len(app.completed_duts) == len(app.active_duts):
                app.log_message("All DUTs have completed their calibration ranges. Ending run early.")
                break

            setpoint_start_time = time.time()
            app.log_message(f"\n--- Setting {sp} Torr ---")
            
            if sp == 0.0:
                app.state_controller.set_pressure(sp)
            else:
                predicted_pos = app._predict_outlet_position(sp)
                app.state_controller.set_pressure(sp, predicted_outlet_pos=predicted_pos)
            
            app.log_message("Waiting for pressure to stabilize...")

            stability_confirmed_time = None
            out_of_tolerance_start_time = None
            relevant_duts = [d for d in app.active_duts if sp in dut_specific_setpoints.get(d['channel'], set())]
            priority_tolerance = min([d['fs'] * 0.005 for d in relevant_duts]) if relevant_duts else app.standard_fs_value * 0.005

            while app.is_calibrating:
                if len(app.state_controller.pressure_history) < 10:
                    time.sleep(0.5)
                    continue

                is_stable = statistics.stdev(app.state_controller.pressure_history) < (app.standard_fs_value * 0.0003)
                stable_pressure = app.state_controller.current_pressure
                if stable_pressure is None:
                    time.sleep(1)
                    continue

                if is_stable:
                    if abs(stable_pressure - sp) <= priority_tolerance:
                        out_of_tolerance_start_time = None
                        if stability_confirmed_time is None: stability_confirmed_time = time.time()
                        if (time.time() - stability_confirmed_time) >= 3.0:
                            app.log_message(f"  Pressure locked at {stable_pressure:.3f} Torr. Proceeding to log.")
                            break
                    else:
                        stability_confirmed_time = None
                        if out_of_tolerance_start_time is None:
                            app.log_message(f"  Pressure stable at {stable_pressure:.3f} Torr, but OUTSIDE tolerance (+/- {priority_tolerance:.4f} Torr).")
                            app.log_message("  Waiting 20 seconds before prompting...")
                            out_of_tolerance_start_time = time.time()
                        elif (time.time() - out_of_tolerance_start_time) >= 20.0:
                            if messagebox.askyesno("Out-of-Tolerance Override", f"Pressure is stable at {stable_pressure:.4f} Torr, but outside tolerance.\n\nAccept this reading?"):
                                break
                            else:
                                out_of_tolerance_start_time = None 
                else:
                    stability_confirmed_time = None
                    out_of_tolerance_start_time = None
                time.sleep(0.5)

            if not app.is_calibrating: continue

            app.log_message("  Starting 5s data log. Locking outlet valve.")
            app.state_controller.hold_outlet_valve = True
            log_start_time = time.time()
            standard_readings = []
            dut_readings = {dut['channel']: [] for dut in app.active_duts}
            while (time.time() - log_start_time) < 5.0 and app.is_calibrating:
                if app.state_controller.current_pressure is not None:
                    standard_readings.append(app.state_controller.current_pressure)
                for dut in app.active_duts:
                    ch = dut['channel']
                    if app.live_dut_pressure_history[ch] and not np.isnan(app.live_dut_pressure_history[ch][-1]):
                        dut_readings[ch].append(app.live_dut_pressure_history[ch][-1])
                time.sleep(0.2)
            app.state_controller.hold_outlet_valve = False
            app.log_message("  Data log complete. Unlocking outlet valve.")

            if not app.is_calibrating: continue

            mean_standard = np.mean(standard_readings) if standard_readings else np.nan
            if np.isnan(mean_standard): continue

            current_outlet_pos = app.state_controller.outlet_valve_pos
            if current_outlet_pos is not None:
                sp_key = round(sp, 3)
                if sp_key not in app.learned_outlet_positions: app.learned_outlet_positions[sp_key] = []
                app.learned_outlet_positions[sp_key].append(current_outlet_pos)
                if len(app.learned_outlet_positions[sp_key]) > 10: app.learned_outlet_positions[sp_key] = app.learned_outlet_positions[sp_key][-10:]
                app.log_message(f"  Updated history for {sp_key:.3f} Torr. Now have {len(app.learned_outlet_positions[sp_key])} data point(s).")

            app.data_storage['Setpoint_Torr'].append(sp)
            app.data_storage['Standard_Pressure_Torr'].append(mean_standard)
            log_line = f"  Logged -> Setpoint: {sp:.2f} | Standard (Avg): {mean_standard:.3f} Torr (took {time.time() - setpoint_start_time:.1f}s)"

            for dut in app.active_duts:
                ch, fs = dut['channel'], dut['fs']
                mean_dut = np.mean(dut_readings.get(ch, [])) if dut_readings.get(ch) else np.nan
                app.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(mean_dut)

                if not np.isnan(mean_dut):
                    log_line += f" | Dev {ch+1} (Avg): {mean_dut:.3f} Torr"
                    if sp in dut_specific_setpoints.get(ch, set()):
                        error = mean_dut - mean_standard
                        if sp not in app.error_plot_data: app.error_plot_data[sp] = {}
                        app.error_plot_data[sp][ch] = {'error': error, 'time': time.time() - app.start_time}
                        if abs(error) > (fs * 0.005):
                            app.log_message(f"  ⚠️ WARNING: Device {ch+1} OUTSIDE tolerance! Error: {error:+.4f} Torr")
                else:
                    log_line += f" | Dev {ch+1}: {'OUT OF RANGE' if sp > fs else 'READ FAILED'}"

            app.log_message(log_line)
            app.after(0, app.update_error_plot)

        if app.is_calibrating:
            app.log_message("\n--- Data Logging Complete. Analyzing results for certificate generation... ---")
            
            output_dir = "Analysis"
            os.makedirs(output_dir, exist_ok=True)
            csv_output_path = os.path.join(output_dir, "calibration_results.csv")
            
            cal_results_df = pd.DataFrame(app.data_storage)
            cal_results_df.to_csv(csv_output_path, index=False)
            app.log_message(f"Data saved to '{csv_output_path}'.")
            app._save_learned_data()

            tech_id = app.tech_id_var.get()

            for dut in app.active_duts:
                app.log_message(f"Checking pass status for DUT {dut['channel']+1}...")
                
                std_points = np.array([s for s, d in zip(cal_results_df['Standard_Pressure_Torr'], cal_results_df[f'Device_{dut["channel"]+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
                dut_points = np.array([d for s, d in zip(cal_results_df['Standard_Pressure_Torr'], cal_results_df[f'Device_{dut["channel"]+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
                
                is_pass = False
                if len(dut_points) > 3:
                    errors = np.abs(dut_points - std_points)
                    max_error = np.max(errors)
                    tolerance = dut['fs'] * 0.005
                    if max_error <= tolerance:
                        is_pass = True

                if is_pass:
                    app.log_message(f"✅ DUT {dut['channel']+1} PASSED. Generating certificate for WIP {dut['wip']}...")
                    cert_path = generate_certificate(dut, cal_results_df, tech_id, app.log_queue)
                    if cert_path:
                        app.generated_certs.append(cert_path)
                else:
                    app.log_message(f"❌ DUT {dut['channel']+1} did not pass tolerance checks. No certificate will be generated.")

    except Exception as e:
        app.log_message(f"FATAL ERROR during logging: {e}")
    finally:
        if app.is_calibrating: app.after(0, app._generate_debug_plot)
        app.is_calibrating = False
        if app.state_controller: app.state_controller.close_valves()
        app.after(10, app.analyze_and_suggest_tuning)

        def enable_buttons():
            app.start_button.config(state=tk.NORMAL)
            app.manual_cal_button.config(state=tk.NORMAL)
            app.e_stop_button.config(state=tk.DISABLED)
            app.set_pressure_button.config(state=tk.NORMAL)
            if app.generated_certs:
                app.upload_to_asana_button.config(state=tk.NORMAL)
        
        app.after(10, enable_buttons)