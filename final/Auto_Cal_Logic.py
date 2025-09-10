# -*- coding: utf-8 -*-
# ==============================================================================
# File:         Auto_Cal_Logic.py
# Author:       Gemini
# Date:         September 5, 2025
# Description:  This module contains the core logic for running the automated
#               calibration sequence. It is called by the main GUI to handle
#               the step-by-step process of setting pressure points, checking
#               for stability, and logging data.
#
# Version Update (Dynamic Plotting):
#   - Added logic to mark DUTs as complete when their FS range is exceeded.
#   - Logs NaN for out-of-range DUTs to ensure final plots are scaled correctly.
# ==============================================================================

import time
import statistics
import numpy as np
import pandas as pd
from tkinter import messagebox
import tkinter as tk


def run_calibration(app):
    """
    The main logic for the automated calibration sequence.

    Args:
        app (CalibrationGUI): The instance of the main application GUI.
    """
    run_start_time = time.time()
    try:
        # Generate a composite list of setpoints from all active devices
        master_setpoints = set()
        for i in range(0, 101, 10): master_setpoints.add(round(app.standard_fs_value * i / 100, 2))
        for dut in app.active_duts:
            for i in range(0, 101, 10): master_setpoints.add(round(dut['fs'] * i / 100, 2))
        setpoints = sorted(list(master_setpoints))
        app.log_message(f"Generated composite setpoints: {setpoints}")

        app.after(0, app._setup_error_plot, setpoints)

        dut_specific_setpoints = {dut['channel']: {round(dut['fs'] * i / 100, 2) for i in range(0, 101, 10)} for dut in app.active_duts}
        
        # --- NEW: Set to track completed DUTs ---
        app.completed_duts.clear()

        for sp in setpoints:
            if not app.is_calibrating: break
            
            # --- NEW: Check which DUTs are still active for this setpoint ---
            # A DUT is considered complete if the setpoint is > 105% of its FS range.
            for dut in app.active_duts:
                if sp > dut['fs'] * 1.05:
                    if dut['channel'] not in app.completed_duts:
                        app.log_message(f"--- DUT {dut['channel']+1} ({dut['fs']} Torr) completed. Hiding trace. ---")
                        app.completed_duts.add(dut['channel'])

            # If all DUTs are complete, we can stop the calibration early
            if len(app.completed_duts) == len(app.active_duts):
                app.log_message("All DUTs have completed their calibration ranges. Ending run early.")
                break

            setpoint_start_time = time.time()
            predicted_pos = app._predict_outlet_position(sp)
            app.log_message(f"\n--- Setting {sp} Torr ---")
            app.state_controller.set_pressure(sp, predicted_outlet_pos=predicted_pos)
            app.log_message("Waiting for pressure to stabilize...")

            # --- Stability Check Loop ---
            stability_confirmed_time = None; out_of_tolerance_start_time = None
            relevant_duts = [d for d in app.active_duts if sp in dut_specific_setpoints.get(d['channel'], set())]
            priority_tolerance = min([d['fs'] * 0.005 for d in relevant_duts]) if relevant_duts else app.standard_fs_value * 0.005

            while app.is_calibrating:
                if len(app.state_controller.pressure_history) < 10: time.sleep(0.5); continue

                is_stable = statistics.stdev(app.state_controller.pressure_history) < (app.standard_fs_value * 0.0003)
                stable_pressure = app.state_controller.current_pressure
                if stable_pressure is None: time.sleep(1); continue

                if is_stable:
                    if abs(stable_pressure - sp) <= priority_tolerance:
                        out_of_tolerance_start_time = None
                        if stability_confirmed_time is None: stability_confirmed_time = time.time()
                        # Require 3 seconds of continuous stability before proceeding
                        if (time.time() - stability_confirmed_time) >= 3.0:
                            app.log_message(f"  Pressure locked at {stable_pressure:.3f} Torr. Proceeding to log.")
                            break
                    else: # Stable, but out of tolerance
                        stability_confirmed_time = None
                        if out_of_tolerance_start_time is None:
                            app.log_message(f"  Pressure stable at {stable_pressure:.3f} Torr, but OUTSIDE tolerance (+/- {priority_tolerance:.4f} Torr).")
                            app.log_message("  Waiting 20 seconds before prompting...")
                            out_of_tolerance_start_time = time.time()
                        elif (time.time() - out_of_tolerance_start_time) >= 20.0:
                            # Ask user to override if it can't settle automatically
                            if messagebox.askyesno("Out-of-Tolerance Override", f"Pressure is stable at {stable_pressure:.4f} Torr, but outside tolerance.\n\nAccept this reading?"):
                                break
                            else:
                                out_of_tolerance_start_time = None # Reset timer
                else:
                    stability_confirmed_time = None; out_of_tolerance_start_time = None
                time.sleep(0.5)

            if not app.is_calibrating: continue

            # --- Data Logging Phase ---
            app.log_message(f"  Starting 5s data log. Locking outlet valve.")
            app.state_controller.hold_outlet_valve = True
            log_start_time = time.time()
            standard_readings = []; dut_readings = {dut['channel']: [] for dut in app.active_duts}
            while (time.time() - log_start_time) < 5.0 and app.is_calibrating:
                if app.state_controller.current_pressure is not None:
                    standard_readings.append(app.state_controller.current_pressure)
                for dut in app.active_duts:
                    ch = dut['channel']
                    if app.live_dut_pressure_history[ch] and not np.isnan(app.live_dut_pressure_history[ch][-1]):
                        dut_readings[ch].append(app.live_dut_pressure_history[ch][-1])
                time.sleep(0.2)
            app.state_controller.hold_outlet_valve = False
            app.log_message(f"  Data log complete. Unlocking outlet valve.")

            if not app.is_calibrating: continue

            # --- Process and Save Logged Data ---
            mean_standard = np.mean(standard_readings) if standard_readings else np.nan
            if np.isnan(mean_standard): continue

            # Learn the outlet position from this successful setpoint
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
                # --- NEW: Log NaN if the setpoint is out of range for the DUT ---
                if sp > fs * 1.05:
                    mean_dut = np.nan
                else:
                    mean_dut = np.mean(dut_readings.get(ch, [])) if dut_readings.get(ch) else np.nan
                
                app.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(mean_dut)

                if not np.isnan(mean_dut):
                    log_line += f" | Dev {ch+1} (Avg): {mean_dut:.3f} Torr"
                    if sp in dut_specific_setpoints.get(ch, set()):
                        error = mean_dut - mean_standard
                        if sp not in app.error_plot_data: app.error_plot_data[sp] = {}
                        app.error_plot_data[sp][ch] = {'error': error, 'time': time.time() - app.start_time}
                        if abs(error) > (fs * 0.005): app.log_message(f"  ⚠️ WARNING: Device {ch+1} OUTSIDE tolerance! Error: {error:+.4f} Torr")
                else: 
                    log_line += f" | Dev {ch+1}: {'OUT OF RANGE' if sp > fs else 'READ FAILED'}"

            app.log_message(log_line)
            app.after(0, app.update_error_plot) # Update plot from main thread

        if app.is_calibrating:
            app.log_message(f"\n--- Data Logging Complete in {time.time() - run_start_time:.1f} seconds. Saving data... ---")
            pd.DataFrame(app.data_storage).to_csv("calibration_results.csv", index=False)
            app.log_message("Data saved to 'calibration_results.csv'.")
            app._save_learned_data()

    except Exception as e:
        app.log_message(f"FATAL ERROR during logging: {e}")
    finally:
        if app.is_calibrating: app.after(0, app._generate_debug_plot)
        app.is_calibrating = False
        if app.state_controller: app.state_controller.close_valves()
        app.after(10, app.analyze_and_suggest_tuning)
        app.after(10, lambda: [
            app.start_button.config(state=tk.NORMAL),
            app.manual_cal_button.config(state=tk.NORMAL),
            app.e_stop_button.config(state=tk.DISABLED),
            app.set_pressure_button.config(state=tk.NORMAL)
        ])
