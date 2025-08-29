# -*- coding: utf-8 -*-
# ==============================================================================
# Script Name: Multi-Device Integrated Calibration Script
# Author: Gemini
# Date: August 28, 2025
#
# Version 64 (Production Ready - Leak Up Recovery):
#   - Added a "Leak Up" detection mode. If the inlet valve is fully closed
#     but pressure is still high, the script now correctly identifies that
#     the outlet is too closed and gently opens it to compensate.
#   - This resolves the final known edge case where an aggressive emergency
#     descent could cause the system to get stuck.
# ==============================================================================

import serial
import serial.tools.list_ports
import time
import pandas as pd
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
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
        self._adaptive_outlet_thread = None
        self.log_queue = log_queue
        
        self.inlet_lock = threading.Lock()
        self.outlet_lock = threading.Lock()

        try:
            self.ser_inlet = serial.Serial(port=inlet_port, baudrate=9600, timeout=1, write_timeout=1)
            self.ser_outlet = serial.Serial(port=outlet_port, baudrate=9600, timeout=1, write_timeout=1)

            self.full_scale_pressure = full_scale_pressure
            self.system_setpoint = 0.0
            self.previous_setpoint = 0.0

            self.pressure_history = collections.deque(maxlen=10)
            
            self.is_connected = True
            self.current_pressure, self.inlet_valve_pos, self.outlet_valve_pos = None, 0.0, 0.0
            self.inlet_pos_history = collections.deque(maxlen=5)
            self.fine_tuning_mode = False
            self.hold_outlet_valve = False
            self.last_log_reason = ""
            self.max_slope_hold = False
            self.oscillation_cooldown = False

        except serial.SerialException as e:
            self.close()
            raise ConnectionError(f"Failed to open controller ports: {e}")

    def _write_to_inlet(self, command):
        with self.inlet_lock:
            if not self.is_connected or not self.ser_inlet: return
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser_inlet.write(full_command)
                self.ser_inlet.flush()
            except serial.SerialTimeoutException:
                self.log_queue.put("ERROR: Write timeout on Inlet Controller!")

    def _query_inlet(self, command):
        with self.inlet_lock:
            if not self.is_connected or not self.ser_inlet: return None
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser_inlet.write(full_command)
                self.ser_inlet.flush()
                response_bytes = self.ser_inlet.readline()
                return response_bytes.decode('ascii', errors='ignore').strip()
            except serial.SerialTimeoutException:
                self.log_queue.put("ERROR: Write timeout on Inlet Controller query!")
                return None

    def _write_to_outlet(self, command):
        with self.outlet_lock:
            if not self.is_connected or not self.ser_outlet: return
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser_outlet.write(full_command)
                self.ser_outlet.flush()
            except serial.SerialTimeoutException:
                self.log_queue.put("ERROR: Write timeout on Outlet Controller!")

    def _query_outlet(self, command):
        with self.outlet_lock:
            if not self.is_connected or not self.ser_outlet: return None
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser_outlet.write(full_command)
                self.ser_outlet.flush()
                response_bytes = self.ser_outlet.readline()
                return response_bytes.decode('ascii', errors='ignore').strip()
            except serial.SerialTimeoutException:
                self.log_queue.put("ERROR: Write timeout on Outlet Controller query!")
                return None

    def start(self):
        if self._polling_thread is None:
            self._stop_event.clear()
            self._polling_thread = threading.Thread(target=self._run_polling_loop, daemon=True)
            self._polling_thread.start()
            self.log_queue.put(">> Controller polling started.")
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
        while not self._stop_event.is_set():
            pressure = self.get_pressure()
            if pressure is not None:
                self.current_pressure = pressure
                self.pressure_history.append(self.current_pressure)
            self.get_valve_positions()
            time.sleep(0.2)
        
    def _run_adaptive_outlet_loop(self):
        while not self._stop_event.is_set():
            if self.hold_outlet_valve:
                time.sleep(1.0)
                continue

            if len(self.pressure_history) < self.pressure_history.maxlen or self.system_setpoint <= 0 or self.current_pressure is None or self.inlet_valve_pos is None:
                time.sleep(1.0)
                continue

            try:
                if len(self.inlet_pos_history) < self.inlet_pos_history.maxlen:
                    continue

                current_outlet_pos = self.outlet_valve_pos
                inlet_valve_pos = self.inlet_valve_pos
                error = self.current_pressure - self.system_setpoint
                new_outlet_pos = current_outlet_pos
                log_reason = "Holding"

                inlet_std_dev = statistics.stdev(self.inlet_pos_history)
                is_inlet_oscillating = inlet_std_dev > 1.0
                
                if is_inlet_oscillating:
                    self.oscillation_cooldown = True 
                    if error > 5.0:
                        new_outlet_pos = current_outlet_pos - 2.0
                        log_reason = f"EMERGENCY DESCENT (Err: {error:+.1f})"
                    else:
                        new_outlet_pos = current_outlet_pos - 0.2
                        log_reason = f"Inlet oscillating (StdDev: {inlet_std_dev:.2f}%)"
                
                elif inlet_valve_pos < 1.0 and error > 0.1:
                    self.oscillation_cooldown = False
                    new_outlet_pos = current_outlet_pos + 0.2
                    log_reason = f"Leak Up Detected (Inlet at {inlet_valve_pos:.1f}%)"

                else:
                    step_size = 0.5
                    is_pressure_stable = statistics.stdev(self.pressure_history) < (0.005 + (self.system_setpoint * 0.001))

                    if is_pressure_stable and error > 0.2:
                        self.oscillation_cooldown = False 
                        new_outlet_pos = current_outlet_pos + step_size
                        log_reason = f"Stuck high (Err: {error:+.2f} Torr)"
                    
                    elif inlet_valve_pos > 25.0 and not self.oscillation_cooldown:
                        inlet_trend = inlet_valve_pos - self.inlet_pos_history[-2]
                        if inlet_trend > 0.1:
                            self.max_slope_hold = True
                            log_reason = "Max Slope Detected"
                        else:
                            new_outlet_pos = current_outlet_pos + step_size
                            log_reason = f"Inlet high ({inlet_valve_pos:.1f}%)"

                    elif inlet_valve_pos < 5.0 and self.system_setpoint > 0:
                        new_outlet_pos = current_outlet_pos - step_size
                        log_reason = f"Inlet low ({inlet_valve_pos:.1f}%)"
                    
                    if self.max_slope_hold:
                        log_reason = "Max Slope Hold"
                        if is_pressure_stable and error > 0.1:
                            self.max_slope_hold = False
                            log_reason += " -> Released"
                        else:
                            new_outlet_pos = current_outlet_pos

                min_clamp = 22.0 
                setpoint_percent = (self.system_setpoint / self.full_scale_pressure) * 100.0
                if setpoint_percent >= 90.0: max_clamp = 26.0
                elif setpoint_percent > 40.0: max_clamp = 30.0
                else: max_clamp = 35.0
                
                clamped_pos = max(min_clamp, min(max_clamp, new_outlet_pos))

                if abs(clamped_pos - current_outlet_pos) > 0.1:
                    if log_reason != self.last_log_reason:
                        self.log_queue.put(f"ADAPT -> Outlet to {clamped_pos:.1f}%. Reason: {log_reason}")
                        self.last_log_reason = log_reason
                    self._write_to_outlet(f"S1 {clamped_pos:.2f}")
                    self._write_to_outlet("D1")
                else:
                    self.last_log_reason = ""

            except (statistics.StatisticsError, AttributeError):
                pass
            
            time.sleep(3.0)

    def set_pressure(self, pressure):
        self.log_queue.put(f">> New system setpoint: {pressure:.3f} Torr")
        self.previous_setpoint = self.system_setpoint
        self.system_setpoint = pressure
        self.pressure_history.clear()
        self.fine_tuning_mode = False
        self.last_log_reason = ""
        self.max_slope_hold = False
        self.oscillation_cooldown = False

        if pressure == 0:
            self.log_queue.put(">> PUMP TO ZERO MODE: Inlet closed, Outlet fully open.")
            self._write_to_inlet("C")
            self._write_to_outlet("S1 100.0")
            self._write_to_outlet("D1")
        else:
            if self.previous_setpoint == 0:
                setpoint_percent = (pressure / self.full_scale_pressure) * 100.0
                if setpoint_percent >= 90.0: initial_outlet_pos = 24.0
                elif setpoint_percent > 40.0: initial_outlet_pos = 28.0
                else: initial_outlet_pos = 35.0
                self.log_queue.put(f">> Moving from zero, dynamic initial outlet: {initial_outlet_pos}%.")
                self._write_to_outlet(f"S1 {initial_outlet_pos:.2f}")
                self._write_to_outlet("D1")
            else:
                self.log_queue.put(f">> Holding current outlet position for new setpoint.")
            
            if self.previous_setpoint == 0:
                self.log_queue.put(">> Allowing system to settle after initial move...")
                time.sleep(3.0)

            inlet_pressure_sp_percent = (pressure / self.full_scale_pressure) * 100.0
            self._write_to_inlet(f"S1 {inlet_pressure_sp_percent:.2f}")
            self._write_to_inlet("D1")

    def get_pressure(self):
        response = self._query_inlet("R5")
        if response:
            try:
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    return (float(match.group()) / 100) * self.full_scale_pressure
            except (ValueError, IndexError): return None
        return None
    
    def get_valve_positions(self):
        try:
            inlet_res = self._query_inlet("R6")
            outlet_res = self._query_outlet("R6")
            
            if inlet_res:
                inlet_match = re.search(r'[+-]?\d+\.?\d*', inlet_res)
                if inlet_match: 
                    pos = float(inlet_match.group())
                    self.inlet_valve_pos = pos
                    self.inlet_pos_history.append(pos)
            
            if outlet_res:
                outlet_match = re.search(r'[+-]?\d+\.?\d*', outlet_res)
                if outlet_match: self.outlet_valve_pos = float(outlet_match.group())

        except (ValueError, IndexError, AttributeError):
            pass
        
    def close_valves(self):
        self.log_queue.put(">> All valves commanded to close.")
        self._write_to_inlet("C")
        self._write_to_outlet("C")
            
    def close(self):
        if self.is_connected:
            self.stop()
            if self.ser_inlet and self.ser_inlet.is_open: self.ser_inlet.close()
            if self.ser_outlet and self.ser_outlet.is_open: self.ser_outlet.close()
            self.is_connected = False
        
# =================================================================================
# Main GUI Class (No changes needed below this line)
# =================================================================================
class CalibrationGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.title("Multi-Device Adaptive Calibration System")
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        self.state_controller = None
        self.daq = None
        self.is_calibrating = False
        self.is_in_manual_mode = False
        self.start_time = 0

        self.dut_colors = ['#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        self.data_storage = {}
        self.log_queue = queue.Queue()
        
        self.manual_focus_device = tk.StringVar(value="std")
        self.manual_focus_channel = None

        self.live_time_history = collections.deque(maxlen=500)
        self.live_inlet_valve_history = collections.deque(maxlen=500)
        self.live_outlet_valve_history = collections.deque(maxlen=500)
        self.live_std_pressure_history = collections.deque(maxlen=500)
        self.live_dut_pressure_history = {i: collections.deque(maxlen=500) for i in range(4)}

        self.manual_trace_time = collections.deque(maxlen=100)
        self.manual_trace_std = collections.deque(maxlen=100)
        self.manual_trace_dut = collections.deque(maxlen=100)
        
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

        tk.Label(config_frame, text="Inlet Controller (Inverse):").grid(row=0, column=0, sticky="w", columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=1, column=0, sticky="e", padx=5)
        self.inlet_com_var = tk.StringVar(self, value="COM9")
        self.inlet_com_combo = ttk.Combobox(config_frame, textvariable=self.inlet_com_var, values=com_ports, width=10)
        self.inlet_com_combo.grid(row=1, column=1, sticky="w")
        
        tk.Label(config_frame, text="Outlet Controller (Direct):").grid(row=2, column=0, sticky="w", pady=(8,0), columnspan=2)
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
            enabled_var = tk.BooleanVar(self, value=True)
            fs_var = tk.StringVar(self); fs_var.set(100.0)
            check = tk.Checkbutton(dut_frame, text="Enable", variable=enabled_var)
            check.grid(row=i, column=1)
            label = tk.Label(dut_frame, text="FS (Torr):")
            label.grid(row=i, column=2, padx=(10,0))
            menu = tk.OptionMenu(dut_frame, fs_var, *valid_ranges)
            menu.grid(row=i, column=3)
            self.dut_widgets.append({'enabled': enabled_var, 'fs': fs_var, 'check': check, 'menu': menu})

        self.plot_term_frame = tk.Frame(self)
        self.plot_term_frame.grid(row=1, column=0, sticky="nsew")
        self.plot_term_frame.rowconfigure(0, weight=1); self.plot_term_frame.columnconfigure(0, weight=1)

        self.fig = plt.figure(figsize=(12, 10))
        gs = gridspec.GridSpec(2, 2, figure=self.fig)
        self.ax_cal = self.fig.add_subplot(gs[0, :])
        self.ax_live_pressure = self.fig.add_subplot(gs[1, 0])
        self.ax_live_valves = self.fig.add_subplot(gs[1, 1])
        self.fig.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_term_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        term_frame = tk.Frame(self.plot_term_frame)
        term_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
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
        self.manual_cal_button = tk.Button(action_frame, text="Manual Cal", command=self.toggle_manual_mode, state=tk.DISABLED, width=15)
        self.manual_cal_button.pack(side=tk.LEFT, padx=5)
        self.start_button = tk.Button(action_frame, text="Start Auto Cal", command=self.start_calibration_thread, state=tk.DISABLED, width=15)
        self.start_button.pack(side=tk.LEFT, padx=5)
        self.e_stop_button = tk.Button(action_frame, text="E-Stop", command=self.e_stop_action, bg="red", fg="white", state=tk.DISABLED, width=15)
        self.e_stop_button.pack(side=tk.LEFT, padx=5)

    def send_manual_command(self, event=None):
        command = self.command_entry.get().strip()
        self.command_entry.delete(0, tk.END)
        if not command: return
        self.log_message(f"> {command}")
        if self.state_controller and self.state_controller.is_connected:
            response = self.state_controller._query_inlet(command)
            if response: self.log_message(f"Response: {response}")
            else: self.log_message("Command sent (no response).")
        else:
            self.log_message("Controller not connected.")
        
    def connect_instruments(self):
        try:
            self.standard_fs_value = float(self.std_fs_var.get())
            self.state_controller = StateMachinePressureController(
                inlet_port=self.inlet_com_var.get(),
                outlet_port=self.outlet_com_var.get(),
                full_scale_pressure=self.standard_fs_value,
                log_queue=self.log_queue
            )
            self.log_message(f"Connected to Controllers on {self.inlet_com_var.get()} & {self.outlet_com_var.get()}.")
            self.state_controller.start()
            
            self.daq = DAQController(self.daq_com_var.get())
            self.log_message(f"Connected to DAQ on {self.daq_com_var.get()}.")

            self.active_duts = [{'channel': i, 'fs': float(w['fs'].get())} for i, w in enumerate(self.dut_widgets) if w['enabled'].get()]
            if not self.active_duts: raise ValueError("At least one DUT must be enabled.")

            self.start_time = time.time()
            self.start_button.config(state=tk.NORMAL); self.e_stop_button.config(state=tk.DISABLED)
            self.manual_cal_button.config(state=tk.NORMAL)
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
        if self.is_calibrating or self.is_in_manual_mode:
            self.is_calibrating = False; self.is_in_manual_mode = False
            self.log_message("\n*** E-STOP ***\nProcess stopped.")
            if self.state_controller: self.state_controller.close_valves()
            self.start_button.config(state=tk.NORMAL); self.manual_cal_button.config(state=tk.NORMAL)
            self.manual_cal_button.config(text="Manual Cal")
            self.e_stop_button.config(state=tk.DISABLED)
        
    def configure_plots(self):
        self.ax_cal.clear(); self.ax_live_pressure.clear(); self.ax_live_valves.clear()

        self.ax_cal.set_title("Calibration Curve: Standard vs. Devices")
        self.ax_cal.set_xlabel("Standard Pressure (Torr)"); self.ax_cal.set_ylabel("Device Pressure (Torr)")
        self.ax_cal.grid(True)
        max_fs = self.standard_fs_value
        for dut in self.active_duts: max_fs = max(max_fs, dut['fs'])
        self.ax_cal.set_xlim([0, max_fs*1.05]); self.ax_cal.set_ylim([0, max_fs*1.05])
        self.ax_cal.plot([0, max_fs], [0, max_fs], 'k--', alpha=0.5, label='Ideal 1:1 Line')
        
        self.ax_live_pressure.set_title("Live Pressure"); self.ax_live_pressure.set_xlabel("Time (s)")
        self.ax_live_pressure.set_ylabel("Pressure (Torr)"); self.ax_live_pressure.grid(True, linestyle=':')
        self.live_std_plot, = self.ax_live_pressure.plot([], [], 'blue', linewidth=2, label='Standard')
        self.live_dut_plots = {}
        for i in range(4):
            line, = self.ax_live_pressure.plot([], [], color=self.dut_colors[i], label=f'DUT {i+1}')
            self.live_dut_plots[i] = line
        self.ax_live_pressure.legend(loc='upper left')

        self.ax_live_valves.set_title("Live Valve Positions"); self.ax_live_valves.set_xlabel("Time (s)")
        self.ax_live_valves.set_ylabel("Valve Position (%)"); self.ax_live_valves.grid(True, linestyle=':')
        self.ax_live_valves.set_ylim([-5, 105])
        self.live_inlet_valve_plot, = self.ax_live_valves.plot([], [], color='green', linestyle='--', label='Inlet Valve')
        self.live_outlet_valve_plot, = self.ax_live_valves.plot([], [], color='red', linestyle=':', label='Outlet Valve')
        self.ax_live_valves.legend(loc='upper left')

        self.canvas.draw_idle()

    def log_message(self, message):
        self.log_queue.put(message)

    def periodic_update(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.terminal_text.config(state=tk.NORMAL); self.terminal_text.insert(tk.END, f"\n{msg}"); self.terminal_text.see(tk.END); self.terminal_text.config(state=tk.DISABLED)
        
        if self.is_in_manual_mode and hasattr(self, 'manual_labels'):
            std_pressure = self.state_controller.current_pressure if self.state_controller else None
            if std_pressure is not None:
                for dut in self.active_duts:
                    ch, fs = dut['channel'], dut['fs']
                    dut_pressure = self.live_dut_pressure_history[ch][-1] if self.live_dut_pressure_history[ch] else None
                    if dut_pressure is not None:
                        diff = dut_pressure - std_pressure
                        self.manual_labels[ch]['diff_var'].set(f"Diff: {diff:+.3f} Torr")
                        
                        inner_tolerance = fs * 0.002
                        outer_tolerance = inner_tolerance * 5
                        fill_percent = 0.0
                        if abs(diff) <= inner_tolerance: fill_percent = 100.0
                        elif abs(diff) < outer_tolerance: fill_percent = 100.0 * (1.0 - (abs(diff) - inner_tolerance) / (outer_tolerance - inner_tolerance))
                        self.manual_labels[ch]['progress']['value'] = fill_percent

            if self.manual_focus_channel is not None:
                self.manual_trace_time.append((self.live_time_history[-1] if self.live_time_history else 0))
                self.manual_trace_std.append(std_pressure)
                self.manual_trace_dut.append(self.live_dut_pressure_history[self.manual_focus_channel][-1] if self.live_dut_pressure_history[self.manual_focus_channel] else None)
                self.manual_std_line.set_data(self.manual_trace_time, self.manual_trace_std)
                self.manual_dut_line.set_data(self.manual_trace_time, self.manual_trace_dut)
                t_max = self.manual_trace_time[-1] if self.manual_trace_time else 0
                self.ax_manual_trace.set_xlim(max(0, t_max - 20), t_max + 1)
                self.ax_manual_trace.relim(); self.ax_manual_trace.autoscale_view(scaley=True)
                self.manual_canvas.draw_idle()

        if self.state_controller and self.state_controller.is_connected:
            current_time = time.time() - self.start_time
            self.live_time_history.append(current_time)
            self.live_inlet_valve_history.append(self.state_controller.inlet_valve_pos)
            self.live_outlet_valve_history.append(self.state_controller.outlet_valve_pos)
            self.live_std_pressure_history.append(self.state_controller.current_pressure)
            
            for i in range(4):
                is_active = any(d['channel'] == i for d in self.active_duts)
                if is_active:
                    voltage = self.daq.read_voltage(i) if self.daq else None
                    fs = [d['fs'] for d in self.active_duts if d['channel'] == i][0]
                    dut_pressure = voltage * (fs / 9.9) if voltage is not None else np.nan
                    self.live_dut_pressure_history[i].append(dut_pressure)
                else:
                    self.live_dut_pressure_history[i].append(np.nan)

            self.live_inlet_valve_plot.set_data(self.live_time_history, self.live_inlet_valve_history)
            self.live_outlet_valve_plot.set_data(self.live_time_history, self.live_outlet_valve_history)
            self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
            
            for i, line in self.live_dut_plots.items():
                line.set_visible(any(d['channel'] == i for d in self.active_duts))
                line.set_data(self.live_time_history, self.live_dut_pressure_history[i])
            
            t_max = self.live_time_history[-1] if self.live_time_history else 0
            self.ax_live_pressure.set_xlim(max(0, t_max - 90), t_max + 1)
            self.ax_live_valves.set_xlim(max(0, t_max - 90), t_max + 1)
            self.ax_live_pressure.relim(); self.ax_live_pressure.autoscale_view(scaley=True)
            self.ax_live_valves.relim(); self.ax_live_valves.autoscale_view(scaley=True)
            self.ax_live_valves.set_ylim([-5, 105])

            self.canvas.draw_idle()
        
        self.after(200, self.periodic_update)

    def toggle_manual_mode(self):
        self.is_in_manual_mode = not self.is_in_manual_mode
        if self.is_in_manual_mode:
            self.manual_cal_button.config(text="Exit Manual Cal")
            self.start_button.config(state=tk.DISABLED)
            self.e_stop_button.config(state=tk.NORMAL)
            self.setup_manual_display()
            threading.Thread(target=self.state_controller.set_pressure, args=(0,), daemon=True).start()
        else:
            self.manual_cal_button.config(text="Manual Cal")
            self.start_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.DISABLED)
            self.teardown_manual_display()
            if self.state_controller: self.state_controller.close_valves()

    def setup_manual_display(self):
        self.canvas.get_tk_widget().grid_remove()
        self.manual_frame = tk.Frame(self.plot_term_frame)
        self.manual_frame.grid(row=0, column=0, sticky="nsew")

        self.manual_frame.rowconfigure(0, weight=1)
        self.manual_frame.columnconfigure(0, weight=1); self.manual_frame.columnconfigure(1, weight=3)

        left_panel = tk.Frame(self.manual_frame, padx=10, pady=10)
        left_panel.grid(row=0, column=0, sticky="nsew")

        tk.Label(left_panel, text="Manual Calibration", font=("Helvetica", 16, "bold")).pack(pady=5, anchor='w')
        
        control_frame = tk.LabelFrame(left_panel, text="Setpoint Control", padx=10, pady=10)
        control_frame.pack(pady=10, fill='x', anchor='n')

        self.manual_setpoint_var = tk.StringVar(value="Current Setpoint: 0.00 Torr")
        tk.Label(control_frame, textvariable=self.manual_setpoint_var, font=("Helvetica", 12)).pack()

        button_frame = tk.Frame(control_frame)
        button_frame.pack(pady=(5,0))
        tk.Button(button_frame, text="Set 0% FS", command=lambda: self.set_manual_pressure(0)).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Set 50% FS", command=lambda: self.set_manual_pressure(0.5)).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Set 100% FS", command=lambda: self.set_manual_pressure(1.0)).pack(side=tk.LEFT, padx=5)

        self.manual_labels = {}
        self.manual_focus_device.set("std")

        device_list_frame = tk.LabelFrame(left_panel, text="Focus Control", padx=10, pady=10)
        device_list_frame.pack(pady=10, fill='both', expand=True, anchor='n')
        
        tk.Radiobutton(device_list_frame, text=f"Standard ({self.standard_fs_value} Torr FS)", variable=self.manual_focus_device, 
                        value="std", command=self.on_manual_focus_change, anchor='w').pack(fill='x')

        for dut in self.active_duts:
            ch, fs = dut['channel'], dut['fs']
            frame = tk.LabelFrame(device_list_frame, text=f"Device {ch+1}", padx=10, pady=5)
            frame.pack(fill="x", expand=True, padx=5, pady=5)
            
            tk.Radiobutton(frame, text=f"Focus on this DUT ({fs} Torr FS)", variable=self.manual_focus_device, 
                            value=f"ch{ch}", command=self.on_manual_focus_change).pack(side=tk.LEFT, padx=5)
            
            diff_var = tk.StringVar(value="Diff: -- Torr")
            tk.Label(frame, textvariable=diff_var, font=("Courier", 12)).pack(side=tk.LEFT, padx=10)
            
            progress = ttk.Progressbar(frame, orient="horizontal", length=150, mode="determinate")
            progress.pack(side=tk.LEFT, padx=10, fill='x', expand=True)
            self.manual_labels[ch] = {'diff_var': diff_var, 'progress': progress}

        self.manual_plot_panel = tk.Frame(self.manual_frame)
        self.manual_plot_panel.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        self.manual_fig, self.ax_manual_trace = plt.subplots(figsize=(8, 6))
        self.ax_manual_trace.set_title("Live Trace: DUT vs. Standard")
        self.ax_manual_trace.set_xlabel("Time (s)"); self.ax_manual_trace.set_ylabel("Pressure (Torr)")
        self.ax_manual_trace.grid(True)
        self.manual_std_line, = self.ax_manual_trace.plot([], [], 'b-', label='Standard')
        self.manual_dut_line, = self.ax_manual_trace.plot([], [], 'g-', label='Focused DUT', linewidth=2)
        self.ax_manual_trace.legend()
        
        self.manual_canvas = FigureCanvasTkAgg(self.manual_fig, master=self.manual_plot_panel)
        self.manual_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.on_manual_focus_change()

    def on_manual_focus_change(self):
        focus_id = self.manual_focus_device.get()
        if focus_id == 'std':
            self.manual_focus_channel = None
            self.log_message(f"Manual control focus set to Standard.")
            if hasattr(self, 'manual_plot_panel'): self.manual_plot_panel.grid_remove()
        else:
            ch = int(focus_id.replace('ch',''))
            self.manual_focus_channel = ch
            fs = [d['fs'] for d in self.active_duts if d['channel'] == ch][0]
            self.log_message(f"Manual control focus set to DUT {ch+1} ({fs} Torr).")
            
            self.manual_trace_time.clear(); self.manual_trace_std.clear(); self.manual_trace_dut.clear()
            if hasattr(self, 'manual_plot_panel'):
                self.ax_manual_trace.set_title(f"Live Trace: DUT {ch+1} vs. Standard")
                self.manual_plot_panel.grid()

    def set_manual_pressure(self, fs_fraction):
        focus_id = self.manual_focus_device.get()
        target_fs = 0

        if focus_id == "std": target_fs = self.standard_fs_value
        else:
            ch = int(focus_id.replace('ch',''))
            target_fs = [d['fs'] for d in self.active_duts if d['channel'] == ch][0]
        
        pressure = target_fs * fs_fraction
        threading.Thread(target=self.state_controller.set_pressure, args=(pressure,), daemon=True).start()
        self.manual_setpoint_var.set(f"Current Setpoint: {pressure:.2f} Torr")

    def teardown_manual_display(self):
        if hasattr(self, 'manual_frame'): self.manual_frame.destroy()
        self.canvas.get_tk_widget().grid()
    
    def start_calibration_thread(self):
        self.is_calibrating = True
        self.log_message("\n--- Starting Automated Data Logging ---")
        self.data_storage = {'Setpoint_Torr': [], 'Standard_Pressure_Torr': []}
        for dut in self.active_duts:
            self.data_storage[f'Device_{dut["channel"]+1}_Pressure_Torr'] = []
        
        self.start_button.config(state=tk.DISABLED); self.manual_cal_button.config(state=tk.DISABLED)
        self.e_stop_button.config(state=tk.NORMAL)
        threading.Thread(target=self.run_calibration, daemon=True).start()

    def run_calibration(self):
        try:
            master_setpoints = set()
            for i in range(0, 101, 10): master_setpoints.add(round(self.standard_fs_value * i / 100, 2))
            for dut in self.active_duts:
                for i in range(0, 101, 10): master_setpoints.add(round(dut['fs'] * i / 100, 2))
            
            setpoints = sorted(list(master_setpoints))
            self.log_message(f"Generated composite setpoints: {setpoints}")

            dut_specific_setpoints = {
                dut['channel']: {round(dut['fs'] * i / 100, 2) for i in range(0, 101, 10)}
                for dut in self.active_duts
            }
            
            for sp in setpoints:
                if not self.is_calibrating: break
                self.log_message(f"\n--- Setting {sp} Torr ---")
                self.state_controller.set_pressure(sp)
                
                self.log_message("Waiting for pressure to stabilize...")
                time.sleep(2.0)
                
                stability_confirmed_time = None
                out_of_tolerance_start_time = None

                relevant_duts = [d for d in self.active_duts if sp in dut_specific_setpoints[d['channel']]]
                priority_tolerance = min([d['fs'] * 0.005 for d in relevant_duts]) if relevant_duts else self.standard_fs_value * 0.005

                while self.is_calibrating:
                    if len(self.state_controller.pressure_history) < 10:
                        time.sleep(0.5)
                        continue

                    is_stable = statistics.stdev(self.state_controller.pressure_history) < (self.standard_fs_value * 0.0002)
                    stable_pressure = self.state_controller.current_pressure
                    
                    if stable_pressure is None:
                        time.sleep(1)
                        continue

                    if is_stable:
                        is_in_tolerance = abs(stable_pressure - sp) <= priority_tolerance

                        if is_in_tolerance:
                            out_of_tolerance_start_time = None 
                            if stability_confirmed_time is None: stability_confirmed_time = time.time()
                            if (time.time() - stability_confirmed_time) >= 5.0:
                                self.log_message(f"  Pressure locked at {stable_pressure:.3f} Torr. Proceeding to log.")
                                break
                        else:
                            stability_confirmed_time = None
                            if out_of_tolerance_start_time is None:
                                self.log_message(f"  Pressure stable at {stable_pressure:.3f} Torr, but OUTSIDE tolerance (+/- {priority_tolerance:.4f} Torr).")
                                self.log_message("  Waiting 20 seconds before prompting...")
                                out_of_tolerance_start_time = time.time()
                            elif (time.time() - out_of_tolerance_start_time) >= 20.0:
                                should_proceed = messagebox.askyesno("Out-of-Tolerance Override", 
                                    f"Pressure is stable at {stable_pressure:.4f} Torr, but outside tolerance ({sp:.4f} +/- {priority_tolerance:.4f} Torr).\n\nAccept this reading?")
                                if should_proceed: break
                                else: out_of_tolerance_start_time = None
                    else:
                        stability_confirmed_time = None; out_of_tolerance_start_time = None
                    
                    time.sleep(0.5)
                
                if not self.is_calibrating: continue
                
                self.log_message(f"  Starting 10s data log. Locking outlet valve.")
                self.state_controller.hold_outlet_valve = True
                
                log_start_time = time.time()
                standard_readings = []
                dut_readings = {dut['channel']: [] for dut in self.active_duts}
                
                while (time.time() - log_start_time) < 10.0 and self.is_calibrating:
                    if self.state_controller.current_pressure is not None:
                        standard_readings.append(self.state_controller.current_pressure)
                    for dut in self.active_duts:
                        ch = dut['channel']
                        if self.live_dut_pressure_history[ch] and not np.isnan(self.live_dut_pressure_history[ch][-1]):
                            dut_readings[ch].append(self.live_dut_pressure_history[ch][-1])
                    time.sleep(0.2)

                self.state_controller.hold_outlet_valve = False
                self.log_message(f"  Data log complete. Unlocking outlet valve.")

                if not self.is_calibrating: continue

                mean_standard = np.mean(standard_readings) if standard_readings else np.nan
                if np.isnan(mean_standard): continue

                self.data_storage['Setpoint_Torr'].append(sp)
                self.data_storage['Standard_Pressure_Torr'].append(mean_standard)
                log_line = f"  Logged -> Setpoint: {sp:.2f} | Standard (Avg): {mean_standard:.3f} Torr"

                for dut in self.active_duts:
                    ch, fs = dut['channel'], dut['fs']
                    mean_dut = np.mean(dut_readings.get(ch, [])) if dut_readings.get(ch) else np.nan
                    self.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(mean_dut)
                    if not np.isnan(mean_dut):
                        log_line += f" | Dev {ch+1} (Avg): {mean_dut:.3f} Torr"
                        if sp in dut_specific_setpoints[ch]:
                            error = mean_dut - mean_standard
                            tolerance = fs * 0.005
                            if abs(error) > tolerance:
                                self.log_message(f"  ⚠️ WARNING: Device {ch+1} OUTSIDE tolerance! Error: {error:+.4f} Torr")
                    else:
                        log_line += f" | Dev {ch+1}: READ FAILED"

                self.log_message(log_line)
                self.after(0, self.update_cal_plot)
            
            if self.is_calibrating:
                self.log_message("\n--- Data Logging Complete. Saving data... ---")
                df = pd.DataFrame(self.data_storage)
                df.to_csv("calibration_results.csv", index=False)
                self.log_message("Data saved to 'calibration_results.csv'.")
            
        except Exception as e:
            self.log_message(f"FATAL ERROR during logging: {e}")
        finally:
            self.is_calibrating = False
            if self.state_controller: self.state_controller.close_valves()
            self.after(10, self.analyze_and_suggest_tuning)
            self.after(10, lambda: [
                self.start_button.config(state=tk.NORMAL),
                self.manual_cal_button.config(state=tk.NORMAL),
                self.e_stop_button.config(state=tk.DISABLED)
            ])

    def show_tuning_suggestions_window(self, suggestions_text):
        win = tk.Toplevel(self); win.title("Tuning Suggestions"); win.geometry("700x600")
        main_frame = tk.Frame(win, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        suggestion_box = scrolledtext.ScrolledText(main_frame, height=10, font=("Courier", 10), wrap=tk.WORD, relief=tk.SOLID, borderwidth=1)
        suggestion_box.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        suggestion_box.insert(tk.END, suggestions_text); suggestion_box.config(state=tk.DISABLED)
        tk.Button(main_frame, text="Close", command=win.destroy).pack()

    def analyze_and_suggest_tuning(self):
        suggestion_parts = ["--- Post-Calibration Tuning Analysis ---"]
        any_suggestions = False

        for dut in self.active_duts:
            ch, fs = dut['channel'], dut['fs']
            std_points = np.array([s for s, d in zip(self.data_storage['Standard_Pressure_Torr'], self.data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d)])
            dut_points = np.array([d for d in self.data_storage[f'Device_{ch+1}_Pressure_Torr'] if not np.isnan(d)])
            if len(dut_points) < 3: continue

            slope, intercept = np.polyfit(std_points, dut_points, 1)
            zero_offset_is_sig = abs(intercept) > (fs * 0.001)
            span_error_is_sig = abs(1.0 - slope) > 0.005
            mid_idx = (np.abs(std_points - (fs * 0.5))).argmin()
            midpoint_error_from_line = dut_points[mid_idx] - (slope * std_points[mid_idx] + intercept)
            linearity_is_sig = abs(midpoint_error_from_line) > (fs * 0.002)

            if not (zero_offset_is_sig or span_error_is_sig or linearity_is_sig):
                suggestion_parts.append(f"\n--- Analysis for DUT {ch+1} ({fs} Torr FS) ---")
                suggestion_parts.append("  ✅ SUCCESS: Device is well-calibrated.")
                continue

            any_suggestions = True
            suggestion_parts.append(f"\n--- Suggestions for DUT {ch+1} ({fs} Torr FS) ---")
            suggestion_parts.append(f"\n[ DIAGNOSIS ]\ny = {slope:.4f}x + {intercept:+.4f}")
            if zero_offset_is_sig: suggestion_parts.append(f" • ZERO OFFSET ERROR: {intercept:+.4f} Torr.")
            if span_error_is_sig: suggestion_parts.append(f" • SPAN (GAIN) ERROR: Gain is {'too high' if slope > 1 else 'too low'} (Slope={slope:.4f}).")
            if linearity_is_sig: suggestion_parts.append(f" • LINEARITY ERROR: Mid-range response {'bows UP' if midpoint_error_from_line > 0 else 'bows DOWN'}.")

            suggestion_parts.append("\n[ RECOMMENDED ADJUSTMENT PLAN ]")
            suggestion_parts.append("\n1. ADJUST ZERO (0% FS)")
            if zero_offset_is_sig: suggestion_parts.append(f"   ➡️ ACTION: Adjust to {'LOWER' if intercept > 0 else 'RAISE'} the reading.")
            suggestion_parts.append("\n2. ADJUST SPAN (100% FS)")
            if span_error_is_sig: suggestion_parts.append(f"   ➡️ ACTION: Adjust to {'LOWER' if slope > 1 else 'RAISE'} the reading.")
            suggestion_parts.append("\n3. RE-CHECK ZERO (Critical Step)")
            suggestion_parts.append("\n4. ADJUST LINEARITY (50% FS)")
            if linearity_is_sig: suggestion_parts.append(f"   ➡️ ACTION: Correct {'upward \"smiling\"' if midpoint_error_from_line > 0 else 'downward \"frowning\"'} bow.")
        
        final_suggestion_text = "\n".join(suggestion_parts)
        self.log_message(final_suggestion_text)
        if any_suggestions: self.show_tuning_suggestions_window(final_suggestion_text)

    def update_cal_plot(self):
        self.ax_cal.clear()
        self.ax_cal.set_title("Calibration Curve: Standard vs. Devices")
        self.ax_cal.set_xlabel("Standard Pressure (Torr)"); self.ax_cal.set_ylabel("Device Pressure (Torr)")
        self.ax_cal.grid(True)

        max_reading = 0
        min_reading = 0

        all_data = []
        if 'Standard_Pressure_Torr' in self.data_storage:
            all_data.extend(self.data_storage['Standard_Pressure_Torr'])
        for dut in self.active_duts:
            key = f'Device_{dut["channel"]+1}_Pressure_Torr'
            if key in self.data_storage:
                all_data.extend(self.data_storage[key])
        
        valid_data = [v for v in all_data if not np.isnan(v)]
        if valid_data:
            max_reading = max(valid_data)
            min_reading = min(valid_data)

        highest_val = max(max_reading, self.standard_fs_value)
        lowest_val = min(0, min_reading)
        
        plot_upper_limit = highest_val * 1.10
        plot_lower_limit = lowest_val - abs(highest_val * 0.05)

        self.ax_cal.set_xlim([plot_lower_limit, plot_upper_limit])
        self.ax_cal.set_ylim([plot_lower_limit, plot_upper_limit])
        
        self.ax_cal.plot([plot_lower_limit, plot_upper_limit], [plot_lower_limit, plot_upper_limit], 'k--', alpha=0.5, label='Ideal 1:1 Line')
        
        std_data = self.data_storage.get('Standard_Pressure_Torr', [])
        for dut in self.active_duts:
            dut_data = self.data_storage.get(f'Device_{dut["channel"]+1}_Pressure_Torr', [])
            valid_points = [(s, d) for s, d in zip(std_data, dut_data) if not (np.isnan(s) or np.isnan(d))]
            if valid_points:
                std_plot, dut_plot = zip(*valid_points)
                color = self.dut_colors[dut['channel']]
                self.ax_cal.plot(std_plot, dut_plot, 'o-', label=f'Device {dut["channel"]+1}', color=color)

        self.ax_cal.legend()
        self.canvas.draw_idle()

    def on_closing(self):
        self.is_calibrating = False; self.is_in_manual_mode = False
        time.sleep(0.3)
        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()
        self.destroy()

if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()