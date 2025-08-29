# -*- coding: utf-8 -*-
# ==============================================================================
# Script Name: Multi-Device Integrated Calibration Script
# Author: Gemini
# Date: August 27, 2025
#
# Version 41 (Zero-Setpoint Safety Delay):
#   - Added a 3-second safety delay when moving from a zero to a non-zero
#     pressure setpoint.
#   - This ensures the outlet valve has sufficient time to move from its fully
#     open state, protecting the pump from atmospheric exposure.
# ==============================================================================

import serial
import serial.tools.list_ports
import time
import pandas as pd
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import scrolledtext, ttk, messagebox
import threading
import queue
import collections
import statistics # Used for oscillation detection

# =================================================================================
# DAQController Class (for Multi-Channel RP2040)
# =================================================================================
class DAQController:
    """Handles communication with the Multi-Channel RP2040 DAQ."""
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, 9600, timeout=2)
            time.sleep(2)
            self.is_connected = True
            self.voltage_history = {i: collections.deque(maxlen=5) for i in range(4)}
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to open DAQ port {port}: {e}")

    def read_voltage(self, channel):
        if not self.is_connected: return None
        try:
            command = f'R{channel}'.encode('ascii')
            self.ser.write(command)
            self.ser.flush()
            response = self.ser.readline().decode('ascii', errors='ignore').strip()
            raw_voltage = float(response)
            self.voltage_history[channel].append(raw_voltage)
            if not self.voltage_history[channel]: return None
            smoothed_voltage = sum(self.voltage_history[channel]) / len(self.voltage_history[channel])
            return smoothed_voltage
        except (ValueError, serial.SerialException):
            return None

    def close(self):
        if self.is_connected and self.ser.is_open:
            self.ser.close()
            self.is_connected = False

# =================================================================================
# StateMachinePressureController Class (FINAL ARCHITECTURE)
# =================================================================================
class StateMachinePressureController:
    """
    Manages a dual-valve system using a hybrid event-driven and adaptive polling scheme.
    """
    def __init__(self, inlet_port, outlet_port, full_scale_pressure, log_queue):
        self.ser_inlet, self.ser_outlet = None, None
        self.is_connected = False
        self._stop_event = threading.Event()
        self._polling_thread = None
        self._adaptive_outlet_thread = None # Thread for the adaptive helper
        self.log_queue = log_queue

        try:
            self.ser_inlet = serial.Serial(port=inlet_port, baudrate=9600, timeout=1)
            self.ser_outlet = serial.Serial(port=outlet_port, baudrate=9600, timeout=1)

            self.full_scale_pressure = full_scale_pressure
            self.system_setpoint = 0.0
            self.previous_setpoint = 0.0 # Track the last setpoint for safety delay

            # For oscillation detection
            self.pressure_history = collections.deque(maxlen=10)
            self.stability_threshold = 0.05 # Torr. If std dev is above this, it's oscillating.
            
            self.is_connected = True
            self.current_pressure, self.inlet_valve_pos, self.outlet_valve_pos = None, 0.0, 0.0

        except serial.SerialException as e:
            self.close()
            raise ConnectionError(f"Failed to open controller ports: {e}")

    def _write_command(self, target_ser, command):
        if not self.is_connected or not target_ser: return
        full_command = (command + '\r').encode('ascii')
        target_ser.write(full_command)
        target_ser.flush()

    def _query_command(self, target_ser, command):
        if not self.is_connected or not target_ser: return None
        self._write_command(target_ser, command)
        response_bytes = target_ser.readline()
        return response_bytes.decode('ascii', errors='ignore').strip()
    
    def query_inlet_command(self, command):
        return self._query_command(self.ser_inlet, command)

    def start(self):
        if self._polling_thread is None:
            self._stop_event.clear()
            # Start the fast, read-only polling loop for the GUI
            self._polling_thread = threading.Thread(target=self._run_polling_loop, daemon=True)
            self._polling_thread.start()
            self.log_queue.put(">> Controller polling started.")
            # Start the slow, adaptive loop for the outlet valve
            self._adaptive_outlet_thread = threading.Thread(target=self._run_adaptive_outlet_loop, daemon=True)
            self._adaptive_outlet_thread.start()
            self.log_queue.put(">> Adaptive outlet helper started.")


    def stop(self):
        self._stop_event.set()
        if self._polling_thread is not None:
            self._polling_thread.join(timeout=2)
        if self._adaptive_outlet_thread is not None:
            self._adaptive_outlet_thread.join(timeout=2)
        self.close_valves()
        self._polling_thread = None
        self._adaptive_outlet_thread = None
    
    def _run_polling_loop(self):
        """This loop ONLY reads data for the GUI and populates the history."""
        while not self._stop_event.is_set():
            pressure = self.get_pressure()
            if pressure is not None:
                self.current_pressure = pressure
                self.pressure_history.append(self.current_pressure)
            self.get_valve_positions()
            time.sleep(0.2) # 5 Hz update rate for GUI
        
    def _run_adaptive_outlet_loop(self):
        """This slow loop makes intelligent adjustments to the outlet valve."""
        while not self._stop_event.is_set():
            # Only run the logic if we have enough data and are not pumping to zero
            if len(self.pressure_history) < self.pressure_history.maxlen or self.system_setpoint <= 0:
                time.sleep(1.0)
                continue

            try:
                # 1. Check for oscillations
                std_dev = statistics.stdev(self.pressure_history)
                is_oscillating = std_dev > self.stability_threshold
                
                current_pos = self.outlet_valve_pos
                new_pos = current_pos

                if is_oscillating:
                    # If oscillating, close the valve slightly to regain stability
                    new_pos = current_pos - 0.5
                    self.log_queue.put(f">> Oscillation detected (stddev={std_dev:.3f}). Closing outlet slightly.")
                else:
                    # If stable, check if we can open the valve more for speed
                    error = abs(self.current_pressure - self.system_setpoint)
                    # Only try to open if we are still far from the setpoint
                    if error > 0.5: # 0.5 Torr deadband
                        new_pos = current_pos + 0.2
                
                # 2. Clamp the new position to the safe operating range
                clamped_pos = max(22.0, min(27.0, new_pos))

                # 3. Only send a command if the position needs to change
                if abs(clamped_pos - current_pos) > 0.1:
                    self._write_command(self.ser_outlet, f"S1 {clamped_pos:.2f}")
                    self._write_command(self.ser_outlet, "D1")

            except statistics.StatisticsError:
                # This can happen if all values in the history are identical
                pass
            
            time.sleep(1.0) # Adjust once per second

    def set_pressure(self, pressure):
        """
        Main entry point for setting the system pressure. Runs ONCE per setpoint change.
        """
        self.log_queue.put(f">> New system setpoint: {pressure:.3f} Torr")
        self.previous_setpoint = self.system_setpoint # Store the old setpoint
        self.system_setpoint = pressure
        self.pressure_history.clear() # Clear history on new setpoint

        # Special case: Pump down to zero
        if pressure == 0:
            self.log_queue.put(">> PUMP TO ZERO MODE: Inlet closed, Outlet fully open.")
            self._write_command(self.ser_inlet, "C")
            self._write_command(self.ser_outlet, "S1 100.0")
            self._write_command(self.ser_outlet, "D1")
        
        # Normal operation: Hold a pressure setpoint
        else:
            # Set a starting "guess" for the outlet valve. The adaptive loop will fine-tune it.
            self._write_command(self.ser_outlet, "S1 25.0")
            self._write_command(self.ser_outlet, "D1")
            
            # SAFETY DELAY: If moving from zero, wait for the outlet valve to move
            if self.previous_setpoint == 0:
                self.log_queue.put(">> Moving from zero, allowing outlet valve 3s to move...")
                time.sleep(3.0)

            # Command the inlet controller to the desired pressure
            inlet_pressure_sp_percent = (pressure / self.full_scale_pressure) * 100.0
            self._write_command(self.ser_inlet, f"S1 {inlet_pressure_sp_percent:.2f}")
            self._write_command(self.ser_inlet, "D1")

    def get_pressure(self):
        response = self._query_command(self.ser_inlet, "R5")
        if response:
            try:
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    percentage_reading = float(match.group())
                    return (percentage_reading / 100) * self.full_scale_pressure
            except (ValueError, IndexError): return None
        return None
    
    def get_valve_positions(self):
        try:
            inlet_res = self._query_command(self.ser_inlet, "R6")
            outlet_res = self._query_command(self.ser_outlet, "R6")
            if inlet_res: self.inlet_valve_pos = float(re.search(r'[+-]?\d+\.?\d*', inlet_res).group())
            if outlet_res: self.outlet_valve_pos = float(re.search(r'[+-]?\d+\.?\d*', outlet_res).group())
            return self.inlet_valve_pos, self.outlet_valve_pos
        except (ValueError, IndexError, AttributeError):
            return self.inlet_valve_pos, self.outlet_valve_pos
        
    def close_valves(self):
        self.log_queue.put(">> All valves commanded to close.")
        self._write_command(self.ser_inlet, "C")
        self._write_command(self.ser_outlet, "C")
            
    def close(self):
        if self.is_connected:
            self.stop()
            if self.ser_inlet and self.ser_inlet.is_open: self.ser_inlet.close()
            if self.ser_outlet and self.ser_outlet.is_open: self.ser_outlet.close()
            self.is_connected = False

# =================================================================================
# Main GUI Class
# =================================================================================
class CalibrationGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.title("Hybrid Pressure Control System")
        self.geometry("1200x900")
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        self.state_controller = None
        self.daq = None
        self.log_queue = queue.Queue()

        self.live_time_history = collections.deque(maxlen=500)
        self.live_inlet_valve_history = collections.deque(maxlen=500)
        self.live_outlet_valve_history = collections.deque(maxlen=500)
        self.live_std_pressure_history = collections.deque(maxlen=500)
        self.live_dut_pressure_history = {i: collections.deque(maxlen=500) for i in range(4)}
        
        self.setup_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after(100, self.periodic_update)

    def setup_ui(self):
        top_config_frame = tk.Frame(self)
        top_config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        top_config_frame.columnconfigure(0, weight=1); top_config_frame.columnconfigure(1, weight=1)

        config_frame = tk.LabelFrame(top_config_frame, text="Configuration", padx=10, pady=10)
        config_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        valid_ranges = sorted([0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 500.0, 1000.0])

        tk.Label(config_frame, text="Inlet Controller (Inverse, COM9):").grid(row=0, column=0, sticky="w", columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=1, column=0, sticky="e", padx=5)
        self.inlet_com_var = tk.StringVar(self, value="COM9")
        self.inlet_com_combo = ttk.Combobox(config_frame, textvariable=self.inlet_com_var, values=com_ports, width=10)
        self.inlet_com_combo.grid(row=1, column=1, sticky="w")
        
        tk.Label(config_frame, text="Outlet Controller (Direct, COM8):").grid(row=2, column=0, sticky="w", pady=(8,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=3, column=0, sticky="e", padx=5)
        self.outlet_com_var = tk.StringVar(self, value="COM8")
        self.outlet_com_combo = ttk.Combobox(config_frame, textvariable=self.outlet_com_var, values=com_ports, width=10)
        self.outlet_com_combo.grid(row=3, column=1, sticky="w")

        tk.Label(config_frame, text="System FS (Torr):").grid(row=4, column=0, sticky="e", padx=5, pady=(8,0))
        self.std_fs_var = tk.StringVar(self); self.std_fs_var.set(100.0)
        self.std_fs_menu = tk.OptionMenu(config_frame, self.std_fs_var, *valid_ranges)
        self.std_fs_menu.grid(row=4, column=1, sticky="w", pady=(8,0))

        tk.Label(config_frame, text="DAQ (RP2040):").grid(row=5, column=0, sticky="w", pady=(10,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=6, column=0, sticky="e", padx=5)
        self.daq_com_var = tk.StringVar(self, value="COM12")
        self.daq_com_combo = ttk.Combobox(config_frame, textvariable=self.daq_com_var, values=com_ports, width=10)
        self.daq_com_combo.grid(row=6, column=1, sticky="w")
        
        dut_frame = tk.LabelFrame(top_config_frame, text="Devices Under Test (DUTs)", padx=10, pady=10)
        dut_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        self.dut_widgets = []
        for i in range(4):
            tk.Label(dut_frame, text=f"Device {i+1} (DAQ Ch {i}):").grid(row=i, column=0, sticky="w")
            enabled_var = tk.BooleanVar(self, value=False)
            fs_var = tk.StringVar(self); fs_var.set(100.0)
            check = tk.Checkbutton(dut_frame, text="Enable", variable=enabled_var)
            check.grid(row=i, column=1)
            label = tk.Label(dut_frame, text="FS (Torr):")
            label.grid(row=i, column=2, padx=(10,0))
            menu = tk.OptionMenu(dut_frame, fs_var, *valid_ranges)
            menu.grid(row=i, column=3)
            self.dut_widgets.append({'enabled': enabled_var, 'fs': fs_var, 'check': check, 'menu': menu})
        self.dut_widgets[0]['enabled'].set(True)

        self.plot_term_frame = tk.Frame(self)
        self.plot_term_frame.grid(row=1, column=0, sticky="nsew")
        self.plot_term_frame.rowconfigure(0, weight=3); self.plot_term_frame.columnconfigure(0, weight=1)
        self.plot_term_frame.rowconfigure(1, weight=1)
        
        self.fig, self.ax_vitals = plt.subplots(1, 1, figsize=(12, 6)); self.fig.tight_layout(pad=3.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_term_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        term_frame = tk.Frame(self.plot_term_frame)
        term_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        term_frame.columnconfigure(0, weight=1)
        self.terminal_text = scrolledtext.ScrolledText(term_frame, height=10, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00")
        self.terminal_text.pack(fill="both", expand=True)
        input_frame = tk.Frame(term_frame)
        input_frame.pack(fill=tk.X, pady=(5, 0))
        tk.Label(input_frame, text="Inlet CMD:").pack(side=tk.LEFT)
        self.command_entry = tk.Entry(input_frame, font=("Courier", 10), bg="#2c2c2c", fg="#00ff00", insertbackground="#00ff00")
        self.command_entry.pack(fill=tk.X, expand=True, side=tk.LEFT)
        self.command_entry.bind("<Return>", self.send_manual_command)

        action_frame = tk.Frame(self)
        action_frame.grid(row=2, column=0, pady=10)
        self.connect_button = tk.Button(action_frame, text="Connect", command=self.connect_instruments, width=15)
        self.connect_button.pack(side=tk.LEFT, padx=5)
        
        tk.Label(action_frame, text="Setpoint (Torr):").pack(side=tk.LEFT, padx=(20, 5))
        self.setpoint_entry = tk.Entry(action_frame, width=10)
        self.setpoint_entry.pack(side=tk.LEFT)
        self.setpoint_entry.bind("<Return>", self.update_setpoint)
        self.set_button = tk.Button(action_frame, text="Set", command=self.update_setpoint, state=tk.DISABLED)
        self.set_button.pack(side=tk.LEFT, padx=5)
        
        self.e_stop_button = tk.Button(action_frame, text="E-Stop & Close Valves", command=self.e_stop_action, bg="red", fg="white", state=tk.DISABLED, width=20)
        self.e_stop_button.pack(side=tk.LEFT, padx=20)

    def send_manual_command(self, event=None):
        command = self.command_entry.get().strip()
        self.command_entry.delete(0, tk.END)
        if not command: return
        self.log_message(f"INLET> {command}")
        if self.state_controller and self.state_controller.is_connected:
            try:
                response = self.state_controller.query_inlet_command(command)
                if response: self.log_message(f"Response: {response}")
                else: self.log_message("Command sent (no response).")
            except Exception as e: self.log_message(f"Error sending command: {e}")
        else: self.log_message("Controllers not connected.")

    def update_setpoint(self, event=None):
        try:
            sp = float(self.setpoint_entry.get())
            if self.state_controller:
                # Run the command logic in a separate thread to keep the GUI responsive
                threading.Thread(target=self.state_controller.set_pressure, args=(sp,), daemon=True).start()
        except ValueError:
            self.log_message("ERROR: Invalid setpoint value.")

    def connect_instruments(self):
        try:
            self.standard_fs_value = float(self.std_fs_var.get())
            self.state_controller = StateMachinePressureController(
                inlet_port=self.inlet_com_var.get(),
                outlet_port=self.outlet_com_var.get(),
                full_scale_pressure=self.standard_fs_value,
                log_queue=self.log_queue
            )
            self.log_message(f"Connected to Inlet on {self.inlet_com_var.get()} and Outlet on {self.outlet_com_var.get()}.")
            self.state_controller.start()
            
            self.daq = DAQController(self.daq_com_var.get())
            self.log_message(f"Connected to DAQ on {self.daq_com_var.get()}.")

            self.active_duts = [{'channel': i, 'fs': float(w['fs'].get())} for i, w in enumerate(self.dut_widgets) if w['enabled'].get()]
            
            self.set_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.NORMAL)
            self.connect_button.config(state=tk.DISABLED)
            self.set_config_state(tk.DISABLED)
            self.configure_plots()
        except (ValueError, ConnectionError) as e:
            self.log_message(f"ERROR: {e}")

    def set_config_state(self, state):
        self.inlet_com_combo.config(state=state); self.outlet_com_combo.config(state=state)
        self.std_fs_menu.config(state=state); self.daq_com_combo.config(state=state)
        for widget_set in self.dut_widgets:
            widget_set['check'].config(state=state); widget_set['menu'].config(state=state)
        
    def e_stop_action(self):
        if self.state_controller:
            # Run the command logic in a separate thread to keep the GUI responsive
            threading.Thread(target=self.state_controller.close_valves, daemon=True).start()
        self.setpoint_entry.delete(0, tk.END)

    def configure_plots(self):
        self.ax_vitals.clear()
        self.ax_vitals.set_title("Live System Vitals"); self.ax_vitals.set_xlabel("Time (s)")
        self.ax_vitals.grid(True, linestyle=':')
        self.ax_pressure = self.ax_vitals
        self.ax_valve = self.ax_vitals.twinx()
        self.ax_pressure.set_ylabel("Pressure (Torr)", color='royalblue')
        self.ax_pressure.tick_params(axis='y', labelcolor='royalblue')
        self.ax_valve.set_ylabel("Valve Position (%)")
        self.ax_valve.set_ylim(-5, 105)
        self.live_std_plot, = self.ax_pressure.plot([], [], 'blue', linewidth=2, label='System Pressure')
        self.live_inlet_valve_plot, = self.ax_valve.plot([], [], color='green', linestyle='--', label='Inlet Valve')
        self.live_outlet_valve_plot, = self.ax_valve.plot([], [], color='red', linestyle=':', label='Outlet Valve')
        self.live_dut_plots = {}
        dut_colors = ['#2ca02c', '#d62728', '#9467bd', '#8c564b']
        for dut in self.active_duts:
            ch = dut['channel']
            line, = self.ax_pressure.plot([], [], color=dut_colors[ch], alpha=0.6, label=f'DUT {ch+1}')
            self.live_dut_plots[ch] = line
        lines, labels = self.ax_pressure.get_legend_handles_labels()
        lines2, labels2 = self.ax_valve.get_legend_handles_labels()
        self.ax_valve.legend(lines + lines2, labels + labels2, loc='upper left')
        self.canvas.draw_idle()

    def log_message(self, message):
        self.log_queue.put(message)

    def periodic_update(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.terminal_text.config(state=tk.NORMAL)
            self.terminal_text.insert(tk.END, f"\n{msg}")
            self.terminal_text.see(tk.END)
            self.terminal_text.config(state=tk.DISABLED)
        
        if self.state_controller and self.state_controller.is_connected:
            current_time = time.time()
            self.live_time_history.append(current_time)
            self.live_std_pressure_history.append(self.state_controller.current_pressure)
            self.live_inlet_valve_history.append(self.state_controller.inlet_valve_pos)
            self.live_outlet_valve_history.append(self.state_controller.outlet_valve_pos)
            for dut in self.active_duts:
                ch, fs = dut['channel'], dut['fs']
                voltage = self.daq.read_voltage(ch) if self.daq else None
                dut_pressure = voltage * (fs / 9.9) if voltage is not None else np.nan
                self.live_dut_pressure_history[ch].append(dut_pressure)
            self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
            self.live_inlet_valve_plot.set_data(self.live_time_history, self.live_inlet_valve_history)
            self.live_outlet_valve_plot.set_data(self.live_time_history, self.live_outlet_valve_history)
            for ch, line in self.live_dut_plots.items():
                line.set_data(self.live_time_history, self.live_dut_pressure_history[ch])
            if self.live_time_history:
                t_max = self.live_time_history[-1]
                t_min = self.live_time_history[0]
                self.ax_pressure.set_xlim(t_min, t_max + 1)
            self.ax_pressure.relim()
            self.ax_pressure.autoscale_view(scaley=True)
            self.canvas.draw_idle()

        self.after(200, self.periodic_update)

    def on_closing(self):
        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()
        self.destroy()

if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()
