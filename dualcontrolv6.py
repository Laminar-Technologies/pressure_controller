# -*- coding: utf-8 -*-
# ==============================================================================
# Script Name: Multi-Device Integrated Calibration Script
# Author: Gemini
# Date: September 3, 2025
#
# Version 89 (Final Production - Logic Correction):
#   - Corrected the core logic for the "Inlet high" trigger based on user
#     feedback. It is now correctly named "Inlet valve overworked".
#   - The trigger now activates when the inlet is >25% OPEN, and the
#     corrective action is to slightly CLOSE the outlet valve to reduce load.
#   - Log message was updated for clarity. This resolves a key instability.
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
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import tkinter as tk
from tkinter import scrolledtext, ttk, messagebox
import threading
import queue
import collections
import statistics # Used for oscillation detection
import json # Added for persistent learning

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
        
        self.hold_all_valves = threading.Event()

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
            self.oscillation_counter = 0
            self.inlet_high_blind_active = False
            self.inlet_high_blind_start_time = 0

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
            if self.hold_all_valves.is_set():
                time.sleep(1.0)
                continue

            if self.hold_outlet_valve:
                time.sleep(1.0)
                continue

            if len(self.pressure_history) < self.pressure_history.maxlen or self.system_setpoint <= 0 or self.current_pressure is None or self.inlet_valve_pos is None:
                time.sleep(1.0)
                continue

            try:
                if self.inlet_high_blind_active and (time.time() - self.inlet_high_blind_start_time) > 10.0:
                    self.log_queue.put(">> Adaptive logic blind deactivated.")
                    self.inlet_high_blind_active = False

                current_outlet_pos = self.outlet_valve_pos
                inlet_valve_pos = self.inlet_valve_pos
                error = self.current_pressure - self.system_setpoint
                new_outlet_pos = current_outlet_pos
                log_reason = "Holding"

                mean_pressure = statistics.mean(self.pressure_history)
                is_near_setpoint = abs(mean_pressure - self.system_setpoint) < (self.full_scale_pressure * 0.02)

                if is_near_setpoint:
                    pressure_oscillation_threshold = (self.system_setpoint * 0.005) + (self.full_scale_pressure * 0.001)
                    pressure_std_dev = statistics.stdev(self.pressure_history)
                    if pressure_std_dev > pressure_oscillation_threshold:
                        self.oscillation_counter += 1
                    else:
                        self.oscillation_counter = 0
                else:
                    self.oscillation_counter = 0

                if self.oscillation_counter >= 2:
                    self.oscillation_cooldown = True
                    if error > (self.full_scale_pressure * 0.05):
                        new_outlet_pos = current_outlet_pos - 2.0
                        log_reason = f"EMERGENCY DESCENT (Err: {error:+.1f})"
                    else:
                        new_outlet_pos = current_outlet_pos - 0.2
                        log_reason = f"Pressure oscillating (StdDev: {statistics.stdev(self.pressure_history):.3f} Torr)"
                    self.oscillation_counter = 0
                
                elif inlet_valve_pos < 1.0 and error > 0.1:
                    self.oscillation_cooldown = False
                    new_outlet_pos = current_outlet_pos + 0.2
                    log_reason = f"Leak Up Detected (Inlet at {inlet_valve_pos:.1f}%)"

                else:
                    step_size = 0.5
                    is_pressure_stable = statistics.stdev(self.pressure_history) < (0.005 + (self.system_setpoint * 0.001))

                    if is_pressure_stable and error > 0.2 and not self.inlet_high_blind_active:
                        self.oscillation_cooldown = False 
                        new_outlet_pos = current_outlet_pos + step_size
                        log_reason = f"Stuck high (Err: {error:+.2f} Torr)"
                    
                    elif inlet_valve_pos < 75.0 and not self.oscillation_cooldown and not self.inlet_high_blind_active:
                        inlet_trend = inlet_valve_pos - self.inlet_pos_history[-2] if len(self.inlet_pos_history) > 1 else 0
                        if inlet_trend < -0.1:
                            self.max_slope_hold = True
                            log_reason = "Max Slope Detected (Opening)"
                        else:
                            new_outlet_pos = current_outlet_pos - step_size
                            percent_open = 100.0 - inlet_valve_pos
                            log_reason = f"Inlet valve overworked ({percent_open:.1f}% open)"

                    elif inlet_valve_pos > 95.0 and self.system_setpoint > 0: # Corresponds to < 5% open
                        new_outlet_pos = current_outlet_pos + step_size
                        percent_open = 100.0 - inlet_valve_pos
                        log_reason = f"Inlet valve near closed ({percent_open:.1f}% open)"
                    
                    if self.max_slope_hold:
                        log_reason = "Max Slope Hold"
                        if is_pressure_stable and error > 0.1:
                            self.max_slope_hold = False
                            log_reason += " -> Released"
                        else:
                            new_outlet_pos = current_outlet_pos
                
                if log_reason == "Holding" and self.inlet_high_blind_active:
                    time.sleep(3.0)
                    continue

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

            except (statistics.StatisticsError, AttributeError, IndexError):
                pass
            
            time.sleep(3.0)

    def set_pressure(self, pressure, predicted_outlet_pos=None):
        self.hold_all_valves.clear()
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
            outlet_was_moved = False
            if predicted_outlet_pos is not None:
                self.log_queue.put(f">> Applying predicted outlet position: {predicted_outlet_pos:.2f}%.")
                self._write_to_outlet(f"S1 {predicted_outlet_pos:.2f}")
                self._write_to_outlet("D1")
                outlet_was_moved = True
            elif self.previous_setpoint == 0:
                setpoint_percent = (pressure / self.full_scale_pressure) * 100.0
                if setpoint_percent >= 90.0: initial_outlet_pos = 24.0
                elif setpoint_percent > 40.0: initial_outlet_pos = 28.0
                else: initial_outlet_pos = 35.0
                self.log_queue.put(f">> Moving from zero, dynamic initial outlet: {initial_outlet_pos}%.")
                self._write_to_outlet(f"S1 {initial_outlet_pos:.2f}")
                self._write_to_outlet("D1")
                outlet_was_moved = True
            else:
                self.log_queue.put(f">> Holding current outlet position for new setpoint.")

            if self.previous_setpoint == 0:
                self.inlet_high_blind_active = True
                self.inlet_high_blind_start_time = time.time()
                self.log_queue.put(">> Adaptive logic blinded for 10s after move from zero.")

            if outlet_was_moved:
                self.log_queue.put(">> Waiting for outlet valve to move...")
                time.sleep(2.0)
                self.get_valve_positions() 
                time.sleep(0.2)

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
        self.hold_all_valves.set()
        self.log_queue.put(">> All valves commanded to close.")
        self._write_to_inlet("C")
        self._write_to_outlet("C")
        time.sleep(0.5)
            
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
        
        self.title("Multi-Device Adaptive Calibration System")
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        self.state_controller = None
        self.daq = None
        self.is_calibrating = False
        self.is_in_manual_mode = False
        self.start_time = 0
        self.after_id = None
        self._manual_learn_thread = None
        self.last_manual_stability_state = False

        self.dut_colors = ['#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        self.data_storage = {}
        self.error_plot_data = {}
        self.log_queue = queue.Queue()
        
        self.learned_positions_file = "learned_outlet_positions.json"
        self.learned_data = {} 
        self.learned_outlet_positions = {}
        try:
            with open(self.learned_positions_file, 'r') as f:
                self.learned_data = json.load(f)
            self.log_message(f"Successfully loaded {len(self.learned_data)} learned configurations.")
        except (FileNotFoundError, json.JSONDecodeError):
            self.log_message("No learned positions file found. Starting fresh.")
            self.learned_data = {}
        
        self.manual_focus_device = tk.StringVar(value="std")
        self.manual_focus_channel = None

        self.live_pressure_var = tk.StringVar(value="---.-- Torr")
        self.live_time_history = collections.deque(maxlen=500)
        self.live_std_pressure_history = collections.deque(maxlen=500)
        self.live_dut_pressure_history = {i: collections.deque(maxlen=500) for i in range(4)}

        self.manual_trace_time = collections.deque(maxlen=200)
        self.manual_trace_std = collections.deque(maxlen=200)
        self.manual_trace_duts = {i: collections.deque(maxlen=200) for i in range(4)}

        self.debug_full_time = []
        self.debug_full_std_pressure = []
        self.debug_full_dut_pressure = {i: [] for i in range(4)}
        self.debug_full_inlet_pos = []
        self.debug_full_outlet_pos = []
        
        self.setup_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after_id = self.after(100, self.periodic_update)

    def setup_ui(self):
        top_config_frame = tk.Frame(self)
        top_config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        top_config_frame.columnconfigure(0, weight=2)
        top_config_frame.columnconfigure(1, weight=2)
        top_config_frame.columnconfigure(2, weight=1)
        top_config_frame.columnconfigure(3, weight=1)

        config_frame = tk.LabelFrame(top_config_frame, text="Configuration", padx=10, pady=10)
        config_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        valid_ranges = sorted([0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 500.0, 1000.0])

        tk.Label(config_frame, text="Inlet Controller (Inverse):").grid(row=0, column=0, sticky="w", columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=1, column=0, sticky="e", padx=5)
        self.inlet_com_var = tk.StringVar(self, value="COM5")
        self.inlet_com_combo = ttk.Combobox(config_frame, textvariable=self.inlet_com_var, values=com_ports, width=10)
        self.inlet_com_combo.grid(row=1, column=1, sticky="w")
        
        tk.Label(config_frame, text="Outlet Controller (Direct):").grid(row=2, column=0, sticky="w", pady=(8,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=3, column=0, sticky="e", padx=5)
        self.outlet_com_var = tk.StringVar(self, value="COM6")
        self.outlet_com_combo = ttk.Combobox(config_frame, textvariable=self.outlet_com_var, values=com_ports, width=10)
        self.outlet_com_combo.grid(row=3, column=1, sticky="w")

        tk.Label(config_frame, text="System FS (Torr):").grid(row=4, column=0, sticky="e", padx=5, pady=(8,0))
        self.std_fs_var = tk.StringVar(self); self.std_fs_var.set(100.0)
        self.std_fs_menu = tk.OptionMenu(config_frame, self.std_fs_var, *valid_ranges)
        self.std_fs_menu.grid(row=4, column=1, sticky="w", pady=(8,0))

        tk.Label(config_frame, text="DAQ (RP2040):").grid(row=5, column=0, sticky="w", pady=(10,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=6, column=0, sticky="e", padx=5)
        self.daq_com_var = tk.StringVar(self, value="COM7")
        self.daq_com_combo = ttk.Combobox(config_frame, textvariable=self.daq_com_var, values=com_ports, width=10)
        self.daq_com_combo.grid(row=6, column=1, sticky="w")

        dut_frame = tk.LabelFrame(top_config_frame, text="Devices Under Test (DUTs)", padx=10, pady=10)
        dut_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 5))
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

        valve_status_frame = tk.LabelFrame(top_config_frame, text="Live Valve Status", padx=10, pady=10)
        valve_status_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 5))
        valve_status_frame.columnconfigure(0, weight=1)
        valve_status_frame.columnconfigure(1, weight=1)

        tk.Label(valve_status_frame, text="Inlet Valve").grid(row=0, column=0)
        tk.Label(valve_status_frame, text="Outlet Valve").grid(row=0, column=1)
        self.inlet_pos_var = tk.StringVar(value="-- %")
        self.outlet_pos_var = tk.StringVar(value="-- %")
        tk.Label(valve_status_frame, textvariable=self.inlet_pos_var, font=("Helvetica", 16, "bold")).grid(row=1, column=0)
        tk.Label(valve_status_frame, textvariable=self.outlet_pos_var, font=("Helvetica", 16, "bold")).grid(row=1, column=1)

        self.inlet_valve_fig, self.ax_inlet_valve = plt.subplots(figsize=(1.2, 1.2), dpi=80)
        self.outlet_valve_fig, self.ax_outlet_valve = plt.subplots(figsize=(1.2, 1.2), dpi=80)

        self.inlet_valve_canvas = FigureCanvasTkAgg(self.inlet_valve_fig, master=valve_status_frame)
        self.outlet_valve_canvas = FigureCanvasTkAgg(self.outlet_valve_fig, master=valve_status_frame)

        self._draw_valve(self.ax_inlet_valve, 0)
        self._draw_valve(self.ax_outlet_valve, 0)

        self.inlet_valve_canvas.get_tk_widget().grid(row=2, column=0, pady=(5,0))
        self.outlet_valve_canvas.get_tk_widget().grid(row=2, column=1, pady=(5,0))
        
        pressure_readout_frame = tk.LabelFrame(top_config_frame, text="Live System Pressure", padx=10, pady=10)
        pressure_readout_frame.grid(row=0, column=3, sticky="nsew")
        pressure_readout_frame.rowconfigure(0, weight=1)
        pressure_readout_frame.columnconfigure(0, weight=1)
        
        pressure_label = tk.Label(pressure_readout_frame, textvariable=self.live_pressure_var, font=("Helvetica", 22, "bold"))
        pressure_label.grid(row=0, column=0, sticky="nsew")

        self.plot_term_frame = tk.Frame(self)
        self.plot_term_frame.grid(row=1, column=0, sticky="nsew")
        self.plot_term_frame.rowconfigure(0, weight=1); self.plot_term_frame.columnconfigure(0, weight=1)
        
        self.fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1], figure=self.fig)
        self.ax_live_pressure = self.fig.add_subplot(gs[0, 0])
        self.ax_error = self.fig.add_subplot(gs[0, 1])
        self.fig.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_term_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        term_frame = tk.Frame(self.plot_term_frame)
        term_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        term_frame.columnconfigure(0, weight=1)
        
        self.terminal_text = scrolledtext.ScrolledText(term_frame, height=8, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00")
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

    def _draw_valve(self, ax, position_percent):
        ax.clear()
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.axis('off')

        valve_body = patches.Circle((0, 0), 1, facecolor='#c0c0c0', edgecolor='black', linewidth=1.5)
        ax.add_patch(valve_body)
        
        normalized_pos = position_percent / 100.0
        scaled_pos = normalized_pos ** 0.5
        
        final_angle_deg = scaled_pos * 90.0
        final_angle_rad = np.deg2rad(final_angle_deg)
        
        ellipse_width = 2 * np.cos(final_angle_rad)
        
        butterfly = patches.Ellipse((0,0), width=ellipse_width, height=2, facecolor='#5a5a5a', edgecolor='black')
        
        transform = transforms.Affine2D().rotate_deg(90 - final_angle_deg) + ax.transData
        butterfly.set_transform(transform)

        ax.add_patch(butterfly)
        
        if ax == self.ax_inlet_valve:
            self.inlet_valve_canvas.draw_idle()
        elif ax == self.ax_outlet_valve:
            self.outlet_valve_canvas.draw_idle()

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
            
            fs_key = str(self.standard_fs_value)
            raw_data = self.learned_data.get(fs_key, {})
            self.learned_outlet_positions = {float(k): v for k, v in raw_data.items()}
            self.log_message(f"Activated learning profile for {fs_key} Torr FS ({len(self.learned_outlet_positions)} points).")

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
            self.log_message("\n*** E-STOP ***\nProcess stopped. Saving learned data...")
            self._save_learned_data()
            
            if self.state_controller: self.state_controller.close_valves()
            self.start_button.config(state=tk.NORMAL); self.manual_cal_button.config(state=tk.NORMAL)
            self.manual_cal_button.config(text="Manual Cal")
            self.e_stop_button.config(state=tk.DISABLED)
        
    def configure_plots(self):
        self.ax_live_pressure.clear(); self.ax_error.clear()
        
        self.ax_live_pressure.set_title("Live Pressure Trace"); self.ax_live_pressure.set_xlabel("Time (s)")
        self.ax_live_pressure.set_ylabel("Pressure (Torr)"); self.ax_live_pressure.grid(True, linestyle=':')
        self.live_std_plot, = self.ax_live_pressure.plot([], [], 'blue', linewidth=2, label='Standard')
        self.live_dut_plots = {}
        for i in range(4):
            line, = self.ax_live_pressure.plot([], [], color=self.dut_colors[i], label=f'DUT {i+1}')
            self.live_dut_plots[i] = line
        self.ax_live_pressure.legend(loc='upper left')

        self.ax_error.set_title("DUT Deviation from Standard")
        self.ax_error.set_xlabel("Error (Torr)")
        self.ax_error.set_ylabel("Setpoint (Torr)")
        self.ax_error.axvline(0, color='k', linestyle='--', alpha=0.5)
        self.ax_error.grid(True, axis='x', linestyle=':')

        self.canvas.draw_idle()

    def _setup_error_plot(self, setpoints):
        self.ax_error.clear()
        self.ax_error.set_title("DUT Deviation")
        self.ax_error.set_xlabel("Error (Torr)")
        self.ax_error.set_ylabel("Setpoint (Torr)")
        
        if setpoints:
            self.ax_error.set_ylim(min(setpoints) - 5, max(setpoints) + 5)
        
        self.ax_error.axvline(0, color='k', linestyle='--', alpha=0.5)
        self.ax_error.grid(True, linestyle=':')
        
        handles = [patches.Patch(color=self.dut_colors[dut['channel']], label=f"DUT {dut['channel']+1}") for dut in self.active_duts]
        self.ax_error.legend(handles=handles, loc='best')

    def log_message(self, message):
        self.log_queue.put(message)

    def periodic_update(self):
        current_time = time.time() - self.start_time if self.start_time > 0 else -1

        while not self.log_queue.empty():
            msg = self.log_queue.get()
            if current_time >= 0:
                timestamp = f"[{current_time: >7.2f}s]"
            else:
                timestamp = "[  --.--s]"
            self.terminal_text.config(state=tk.NORMAL)
            self.terminal_text.insert(tk.END, f"\n{timestamp} {msg}")
            self.terminal_text.see(tk.END)
            self.terminal_text.config(state=tk.DISABLED)
        
        if self.state_controller and self.state_controller.is_connected:
            std_pressure = self.state_controller.current_pressure
            inlet_pos = self.state_controller.inlet_valve_pos
            outlet_pos = self.state_controller.outlet_valve_pos
            
            if std_pressure is not None:
                self.live_pressure_var.set(f"{std_pressure:.3f} Torr")
            else:
                self.live_pressure_var.set("---.-- Torr")

            self.live_time_history.append(current_time)
            self.live_std_pressure_history.append(std_pressure)
            self.manual_trace_time.append(current_time)
            self.manual_trace_std.append(std_pressure)

            if self.is_calibrating:
                self.debug_full_time.append(current_time)
                self.debug_full_std_pressure.append(std_pressure)
                self.debug_full_inlet_pos.append(inlet_pos)
                self.debug_full_outlet_pos.append(outlet_pos)

            try:
                display_inlet_pos = 100.0 - inlet_pos if inlet_pos is not None else 0.0
                display_outlet_pos = outlet_pos if outlet_pos is not None else 0.0
                self.inlet_pos_var.set(f"{display_inlet_pos:.1f} %")
                self.outlet_pos_var.set(f"{display_outlet_pos:.1f} %")
                self._draw_valve(self.ax_inlet_valve, display_inlet_pos)
                self._draw_valve(self.ax_outlet_valve, display_outlet_pos)
            except Exception:
                pass
            
            for i in range(4):
                is_active = any(d['channel'] == i for d in self.active_duts)
                dut_pressure = np.nan
                if is_active:
                    voltage = self.daq.read_voltage(i) if self.daq else None
                    if voltage is not None:
                        fs = [d['fs'] for d in self.active_duts if d['channel'] == i][0]
                        dut_pressure = voltage * (fs / 9.9)
                
                self.live_dut_pressure_history[i].append(dut_pressure)
                self.manual_trace_duts[i].append(dut_pressure)
                if self.is_calibrating:
                    self.debug_full_dut_pressure[i].append(dut_pressure)

            self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
            for i, line in self.live_dut_plots.items():
                line.set_visible(any(d['channel'] == i for d in self.active_duts))
                line.set_data(self.live_time_history, self.live_dut_pressure_history[i])
            t_max_main = self.live_time_history[-1] if self.live_time_history else 0
            self.ax_live_pressure.set_xlim(max(0, t_max_main - 90), t_max_main + 1)
            self.ax_live_pressure.relim(); self.ax_live_pressure.autoscale_view(scaley=True)
            self.canvas.draw_idle()

            if hasattr(self, 'manual_frame'): 
                t_max_manual = self.manual_trace_time[-1] if self.manual_trace_time else 0
                
                for i, plot_data in enumerate(self.manual_quadrant_lines):
                    ax = plot_data['ax']
                    if ax.get_visible():
                        lines = self.manual_quadrant_lines[i]
                        lines['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                        lines['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[i])
                        ax.relim()
                        ax.autoscale_view(scaley=True)
                self.axs_manual_quad.flat[0].set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                self.manual_quad_canvas.draw_idle()

                ch = self.manual_focus_channel
                if ch is not None:
                    self.manual_single_lines['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                    self.manual_single_lines['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[ch])
                    self.ax_manual_single.set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                    self.ax_manual_single.relim(); self.ax_manual_single.autoscale_view(scaley=True)
                    self.manual_single_canvas.draw_idle()

        self.after_id = self.after(200, self.periodic_update)

    def toggle_manual_mode(self):
        self.is_in_manual_mode = not self.is_in_manual_mode
        if self.is_in_manual_mode:
            self.manual_cal_button.config(text="Exit Manual Cal")
            self.start_button.config(state=tk.DISABLED)
            self.e_stop_button.config(state=tk.NORMAL)
            self.setup_manual_display()

            self.last_manual_stability_state = False
            self.manual_point_learned = threading.Event()
            self._manual_learn_thread = threading.Thread(target=self._run_manual_learning_loop, daemon=True)
            self._manual_learn_thread.start()
            
            self.set_manual_pressure(0)
        else:
            self.manual_cal_button.config(text="Manual Cal")
            self.start_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.DISABLED)
            if self._manual_learn_thread is not None:
                self._manual_learn_thread.join(timeout=0.5)
            self.teardown_manual_display()
            if self.state_controller: self.state_controller.close_valves()

    def _run_manual_learning_loop(self):
        while self.is_in_manual_mode:
            time.sleep(0.5)
            if self.manual_learn_target is None or self.manual_point_learned.is_set():
                continue

            history = self.state_controller.pressure_history
            if len(history) < history.maxlen:
                continue
            
            try:
                current_pressure = self.state_controller.current_pressure
                if current_pressure is None: continue

                is_stable = statistics.stdev(history) < (self.standard_fs_value * 0.0003)
                is_on_target = abs(current_pressure - self.manual_learn_target) < (self.standard_fs_value * 0.0015)
                is_stable_now = is_stable and is_on_target

                if is_stable_now and not self.last_manual_stability_state:
                    self.log_message(f"** System has stabilized at {current_pressure:.3f} Torr. **")
                    
                    sp = self.manual_learn_target
                    pos = self.state_controller.outlet_valve_pos

                    if pos is not None and sp > 0:
                        sp_key = round(sp, 3)
                        if sp_key not in self.learned_outlet_positions:
                            self.learned_outlet_positions[sp_key] = []
                        
                        self.learned_outlet_positions[sp_key].append(pos)
                        
                        if len(self.learned_outlet_positions[sp_key]) > 10:
                            self.learned_outlet_positions[sp_key] = self.learned_outlet_positions[sp_key][-10:]
                        
                        self.log_message(f"Auto-learned manual point for {sp_key:.3f} Torr. Now have {len(self.learned_outlet_positions[sp_key])} data point(s).")
                        self.manual_point_learned.set()
                
                self.last_manual_stability_state = is_stable_now
            except (statistics.StatisticsError, TypeError):
                continue

    def setup_manual_display(self):
        self.canvas.get_tk_widget().grid_remove()
        self.manual_frame = tk.Frame(self.plot_term_frame)
        self.manual_frame.grid(row=0, column=0, sticky="nsew")

        self.manual_frame.rowconfigure(0, weight=1)
        self.manual_frame.columnconfigure(0, weight=1); self.manual_frame.columnconfigure(1, weight=3)

        left_panel = tk.Frame(self.manual_frame, padx=10, pady=10)
        left_panel.grid(row=0, column=0, sticky="nsew")

        tk.Label(left_panel, text="Manual Calibration", font=("Helvetica", 16, "bold")).pack(pady=5, anchor='w')
        tk.Label(left_panel, textvariable=self.live_pressure_var, font=("Helvetica", 20, "bold"), fg="#00529B").pack(pady=(0,10), anchor='w')
        
        control_frame = tk.LabelFrame(left_panel, text="Setpoint Control", padx=10, pady=10)
        control_frame.pack(pady=10, fill='x', anchor='n')

        self.manual_setpoint_var = tk.StringVar(value="Current Setpoint: 0.00 Torr")
        tk.Label(control_frame, textvariable=self.manual_setpoint_var, font=("Helvetica", 12)).pack()

        button_frame = tk.Frame(control_frame)
        button_frame.pack(pady=(10,5))
        tk.Button(button_frame, text="Set 0% FS", command=lambda: self.set_manual_pressure(0)).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Set 10% FS", command=lambda: self.set_manual_pressure(0.1)).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Set 50% FS", command=lambda: self.set_manual_pressure(0.5)).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Set 100% FS", command=lambda: self.set_manual_pressure(1.0)).pack(side=tk.LEFT, padx=5)

        custom_sp_frame = tk.Frame(control_frame)
        custom_sp_frame.pack(pady=(5,0), fill='x')
        tk.Label(custom_sp_frame, text="Custom (Torr):").pack(side=tk.LEFT)
        self.custom_pressure_entry = tk.Entry(custom_sp_frame, width=10)
        self.custom_pressure_entry.pack(side=tk.LEFT, padx=5, expand=True, fill='x')
        tk.Button(custom_sp_frame, text="Set", command=self._set_custom_pressure).pack(side=tk.LEFT, padx=5)

        self.manual_labels = {}
        self.manual_focus_device.set("std")

        device_list_frame = tk.LabelFrame(left_panel, text="Focus Control", padx=10, pady=10)
        device_list_frame.pack(pady=10, fill='both', expand=True, anchor='n')
        
        tk.Radiobutton(device_list_frame, text=f"Standard ({self.standard_fs_value} Torr FS)", variable=self.manual_focus_device, 
                        value="std", command=self.on_manual_focus_change, anchor='w').pack(fill='x')

        for dut in self.active_duts:
            ch, fs = dut['channel'], dut['fs']
            frame = tk.Frame(device_list_frame)
            frame.pack(fill="x", expand=True, padx=5, pady=5)
            
            tk.Radiobutton(frame, text=f"Focus on DUT {ch+1} ({fs} Torr FS)", variable=self.manual_focus_device, 
                            value=f"ch{ch}", command=self.on_manual_focus_change).pack(side=tk.LEFT, padx=5)
            
        self.manual_plot_panel = tk.Frame(self.manual_frame)
        self.manual_plot_panel.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Create Quadrant plot for Standard focus
        self.manual_quad_fig, self.axs_manual_quad = plt.subplots(2, 2, figsize=(9, 7), sharex=True, sharey=False)
        self.manual_quadrant_lines = []
        for i, ax in enumerate(self.axs_manual_quad.flat):
            ax.set_title(f'DUT {i+1} vs. Standard', fontsize=10)
            std_line, = ax.plot([], [], 'b-', label='Standard')
            dut_line, = ax.plot([], [], color=self.dut_colors[i], label=f'DUT {i+1}')
            ax.grid(True, linestyle=':')
            ax.legend(fontsize='small')
            self.manual_quadrant_lines.append({'std': std_line, 'dut': dut_line, 'ax': ax})
        self.manual_quad_fig.tight_layout()
        self.manual_quad_canvas = FigureCanvasTkAgg(self.manual_quad_fig, master=self.manual_plot_panel)

        # Create Single plot for individual DUT focus
        self.manual_single_fig, self.ax_manual_single = plt.subplots(figsize=(9, 7))
        std_line, = self.ax_manual_single.plot([], [], 'b-', label='Standard', linewidth=2)
        dut_line, = self.ax_manual_single.plot([], [], 'g-', label='Focused DUT')
        self.manual_single_lines = {'std': std_line, 'dut': dut_line}
        self.ax_manual_single.grid(True)
        self.ax_manual_single.legend()
        self.manual_single_canvas = FigureCanvasTkAgg(self.manual_single_fig, master=self.manual_plot_panel)
        
        self.on_manual_focus_change()

    def on_manual_focus_change(self):
        focus_id = self.manual_focus_device.get()
        self.manual_trace_time.clear()
        self.manual_trace_std.clear()
        for i in range(4): self.manual_trace_duts[i].clear()

        if focus_id == 'std':
            self.manual_focus_channel = None
            self.log_message(f"Manual control focus set to Standard (Quadrant View).")
            self.manual_single_canvas.get_tk_widget().pack_forget()
            self.manual_quad_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            for i, plot_data in enumerate(self.manual_quadrant_lines):
                is_active = any(d['channel'] == i for d in self.active_duts)
                plot_data['ax'].set_visible(is_active)
            self.manual_quad_canvas.draw_idle()
        else:
            ch = int(focus_id.replace('ch',''))
            self.manual_focus_channel = ch
            fs = [d['fs'] for d in self.active_duts if d['channel'] == ch][0]
            self.log_message(f"Manual control focus set to DUT {ch+1} ({fs} Torr).")
            self.manual_quad_canvas.get_tk_widget().pack_forget()
            self.manual_single_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            self.ax_manual_single.set_title(f"Live Trace: DUT {ch+1} vs. Standard")
            self.manual_single_lines['dut'].set_color(self.dut_colors[ch])
            self.manual_single_lines['dut'].set_label(f'DUT {ch+1}')
            self.ax_manual_single.legend()
            self.manual_single_canvas.draw_idle()
        
    def _set_custom_pressure(self):
        try:
            pressure_str = self.custom_pressure_entry.get()
            pressure = float(pressure_str)
            if not (0 <= pressure <= self.standard_fs_value):
                messagebox.showerror("Invalid Input", f"Pressure must be between 0 and {self.standard_fs_value} Torr.")
                return
            
            self.manual_learn_target = pressure
            self.manual_point_learned.clear()
            self.last_manual_stability_state = False
            
            predicted_pos = self._predict_outlet_position(pressure)
            threading.Thread(target=self.state_controller.set_pressure, args=(pressure, predicted_pos), daemon=True).start()
            self.manual_setpoint_var.set(f"Current Setpoint: {pressure:.3f} Torr")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for the pressure.")

    def set_manual_pressure(self, fs_fraction):
        focus_id = self.manual_focus_device.get()
        target_fs = self.standard_fs_value

        if focus_id != "std":
            ch = int(focus_id.replace('ch',''))
            target_fs = [d['fs'] for d in self.active_duts if d['channel'] == ch][0]
        
        pressure = target_fs * fs_fraction
        
        self.manual_learn_target = pressure
        self.manual_point_learned.clear()
        self.last_manual_stability_state = False
        
        predicted_pos = self._predict_outlet_position(pressure)
        threading.Thread(target=self.state_controller.set_pressure, args=(pressure, predicted_pos), daemon=True).start()
        self.manual_setpoint_var.set(f"Current Setpoint: {pressure:.3f} Torr")

    def teardown_manual_display(self):
        if hasattr(self, 'manual_frame'): self.manual_frame.destroy()
        self.canvas.get_tk_widget().grid()
    
    def start_calibration_thread(self):
        self.is_calibrating = True
        
        self.debug_full_time.clear()
        self.debug_full_std_pressure.clear()
        self.debug_full_inlet_pos.clear()
        self.debug_full_outlet_pos.clear()
        for i in range(4):
            self.debug_full_dut_pressure[i].clear()
        
        self.log_message("\n--- Starting Automated Data Logging ---")
        self.data_storage = {'Setpoint_Torr': [], 'Standard_Pressure_Torr': []}
        for dut in self.active_duts:
            self.data_storage[f'Device_{dut["channel"]+1}_Pressure_Torr'] = []
        self.error_plot_data.clear()
        
        self.start_button.config(state=tk.DISABLED); self.manual_cal_button.config(state=tk.DISABLED)
        self.e_stop_button.config(state=tk.NORMAL)
        threading.Thread(target=self.run_calibration, daemon=True).start()
    
    def _predict_outlet_position(self, target_sp):
        """Averages historical data and interpolates to predict the best outlet position."""
        if not self.learned_outlet_positions:
            return None

        avg_positions = {}
        for sp, pos_list in self.learned_outlet_positions.items():
            if pos_list and isinstance(pos_list, list):
                avg_positions[float(sp)] = sum(pos_list) / len(pos_list)
        
        if not avg_positions:
            return None

        known_sps = sorted(avg_positions.keys())

        if target_sp in known_sps:
            return avg_positions[target_sp]

        if len(known_sps) < 2:
            closest_sp = min(known_sps, key=lambda sp: abs(sp - target_sp))
            return avg_positions[closest_sp]

        if target_sp < known_sps[0]:
            sp1, sp2 = known_sps[0], known_sps[1]
        elif target_sp > known_sps[-1]:
            sp1, sp2 = known_sps[-2], known_sps[-1]
        else:
            sp_high = min(sp for sp in known_sps if sp > target_sp)
            sp_low = max(sp for sp in known_sps if sp < target_sp)
            sp1, sp2 = sp_low, sp_high

        pos1 = avg_positions[sp1]
        pos2 = avg_positions[sp2]

        if sp2 == sp1: return pos1

        fraction = (target_sp - sp1) / (sp2 - sp1)
        predicted_pos = pos1 + fraction * (pos2 - pos1)
        return predicted_pos

    def run_calibration(self):
        run_start_time = time.time()
        try:
            master_setpoints = set()
            for i in range(0, 101, 10): master_setpoints.add(round(self.standard_fs_value * i / 100, 2))
            for dut in self.active_duts:
                for i in range(0, 101, 10): master_setpoints.add(round(dut['fs'] * i / 100, 2))
            
            setpoints = sorted(list(master_setpoints))
            self.log_message(f"Generated composite setpoints: {setpoints}")

            self.after(0, self._setup_error_plot, setpoints)

            dut_specific_setpoints = {
                dut['channel']: {round(dut['fs'] * i / 100, 2) for i in range(0, 101, 10)}
                for dut in self.active_duts
            }
            
            for sp in setpoints:
                if not self.is_calibrating: break
                setpoint_start_time = time.time()
                
                predicted_pos = self._predict_outlet_position(sp)
                self.log_message(f"\n--- Setting {sp} Torr ---")
                self.state_controller.set_pressure(sp, predicted_outlet_pos=predicted_pos)
                
                self.log_message("Waiting for pressure to stabilize...")
                
                stability_confirmed_time = None
                out_of_tolerance_start_time = None

                relevant_duts = [d for d in self.active_duts if sp in dut_specific_setpoints.get(d['channel'], set())]
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
                            if (time.time() - stability_confirmed_time) >= 3.0:
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
                
                self.log_message(f"  Starting 5s data log. Locking outlet valve.")
                self.state_controller.hold_outlet_valve = True
                
                log_start_time = time.time()
                standard_readings = []
                dut_readings = {dut['channel']: [] for dut in self.active_duts}
                
                while (time.time() - log_start_time) < 5.0 and self.is_calibrating:
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

                log_completion_time = time.time() - self.start_time
                mean_standard = np.mean(standard_readings) if standard_readings else np.nan
                if np.isnan(mean_standard): continue

                current_outlet_pos = self.state_controller.outlet_valve_pos
                if current_outlet_pos is not None:
                    sp_key = round(sp, 3)
                    if sp_key not in self.learned_outlet_positions:
                        self.learned_outlet_positions[sp_key] = []
                    
                    self.learned_outlet_positions[sp_key].append(current_outlet_pos)
                    
                    if len(self.learned_outlet_positions[sp_key]) > 10:
                        self.learned_outlet_positions[sp_key] = self.learned_outlet_positions[sp_key][-10:]
                    
                    self.log_message(f"  Updated history for {sp_key:.3f} Torr. Now have {len(self.learned_outlet_positions[sp_key])} data point(s).")

                self.data_storage['Setpoint_Torr'].append(sp)
                self.data_storage['Standard_Pressure_Torr'].append(mean_standard)
                
                setpoint_duration = time.time() - setpoint_start_time
                log_line = f"  Logged -> Setpoint: {sp:.2f} | Standard (Avg): {mean_standard:.3f} Torr (took {setpoint_duration:.1f}s)"

                for dut in self.active_duts:
                    ch, fs = dut['channel'], dut['fs']
                    mean_dut = np.mean(dut_readings.get(ch, [])) if dut_readings.get(ch) else np.nan
                    self.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(mean_dut)
                    if not np.isnan(mean_dut):
                        log_line += f" | Dev {ch+1} (Avg): {mean_dut:.3f} Torr"
                        
                        if sp in dut_specific_setpoints.get(ch, set()):
                            error = mean_dut - mean_standard
                            if sp not in self.error_plot_data: self.error_plot_data[sp] = {}
                            self.error_plot_data[sp][ch] = {'error': error, 'time': log_completion_time}
                            
                            tolerance = fs * 0.005
                            if abs(error) > tolerance:
                                self.log_message(f"  ⚠️ WARNING: Device {ch+1} OUTSIDE tolerance! Error: {error:+.4f} Torr")
                    else:
                        log_line += f" | Dev {ch+1}: READ FAILED"

                self.log_message(log_line)
                self.after(0, self.update_error_plot)
            
            if self.is_calibrating:
                total_duration = time.time() - run_start_time
                self.log_message(f"\n--- Data Logging Complete in {total_duration:.1f} seconds. Saving data... ---")
                df = pd.DataFrame(self.data_storage)
                df.to_csv("calibration_results.csv", index=False)
                self.log_message("Data saved to 'calibration_results.csv'.")
                self._save_learned_data()

        except Exception as e:
            self.log_message(f"FATAL ERROR during logging: {e}")
        finally:
            if self.is_calibrating:
                 self.after(0, self._generate_debug_plot)
            self.is_calibrating = False
            if self.state_controller: self.state_controller.close_valves()
            self.after(10, self.analyze_and_suggest_tuning)
            self.after(10, lambda: [
                self.start_button.config(state=tk.NORMAL),
                self.manual_cal_button.config(state=tk.NORMAL),
                self.e_stop_button.config(state=tk.DISABLED)
            ])
    
    def _save_learned_data(self):
        if not self.state_controller or not hasattr(self, 'standard_fs_value'):
            self.log_message("Cannot save learning data: System not fully initialized.")
            return
        try:
            fs_key = str(self.standard_fs_value)
            self.learned_data[fs_key] = {str(k): v for k, v in self.learned_outlet_positions.items()}
            with open(self.learned_positions_file, 'w') as f:
                json.dump(self.learned_data, f, indent=4)
            self.log_message(f"Saved {len(self.learned_outlet_positions)} learned points for {fs_key} Torr FS.")
        except Exception as e:
            self.log_message(f"ERROR: Could not save learned positions: {e}")

    def _generate_debug_plot(self):
        """Creates and saves a detailed plot of the entire calibration run."""
        self.log_message("Generating full-run debug plot...")
        try:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
            fig.suptitle('Full Calibration Run - Debug Trace', fontsize=16)

            ax1.set_title('Pressure vs. Time')
            ax1.set_ylabel('Pressure (Torr)')
            ax1.grid(True, linestyle=':')
            ax1.plot(self.debug_full_time, self.debug_full_std_pressure, label='Standard', color='blue', linewidth=2)
            for i, dut in enumerate(self.active_duts):
                ch = dut['channel']
                ax1.plot(self.debug_full_time, self.debug_full_dut_pressure[ch], label=f'DUT {ch+1}', color=self.dut_colors[ch], alpha=0.8)
            ax1.legend()

            ax2.set_title('Valve Position vs. Time')
            ax2.set_ylabel('Position (% Open)')
            ax2.set_xlabel('Time (s)')
            ax2.grid(True, linestyle=':')
            ax2.set_ylim(-5, 105)
            
            inlet_openness = [100 - pos if pos is not None else np.nan for pos in self.debug_full_inlet_pos]
            outlet_openness = [pos if pos is not None else np.nan for pos in self.debug_full_outlet_pos]
            ax2.plot(self.debug_full_time, inlet_openness, label='Inlet Valve (% Open)', color='green', linestyle='--')
            ax2.plot(self.debug_full_time, outlet_openness, label='Outlet Valve (% Open)', color='red', linestyle=':')
            ax2.legend()
            
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            fig.savefig('calibration_debug_trace.png', dpi=150)
            plt.close(fig)
            self.log_message("Debug plot saved to 'calibration_debug_trace.png'")
        except Exception as e:
            self.log_message(f"ERROR: Could not generate debug plot: {e}")

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
            std_points = np.array([s for s, d in zip(self.data_storage['Standard_Pressure_Torr'], self.data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
            dut_points = np.array([d for s, d in zip(self.data_storage['Standard_Pressure_Torr'], self.data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
            if len(dut_points) < 3: continue

            slope, intercept = np.polyfit(std_points, dut_points, 1)
            zero_offset_is_sig = abs(intercept) > (fs * 0.001)
            span_error_is_sig = abs(1.0 - slope) > 0.005
            
            non_linearity_points = []
            for std_p, dut_p in zip(std_points, dut_points):
                linear_p = slope * std_p + intercept
                non_linearity_points.append(dut_p - linear_p)
            
            if non_linearity_points:
                max_non_linearity = max(np.abs(non_linearity_points))
                linearity_is_sig = max_non_linearity > (fs * 0.002)
                mid_idx = np.abs(np.array(non_linearity_points)).argmax()
                midpoint_error_from_line = non_linearity_points[mid_idx]
            else:
                linearity_is_sig = False

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

    def update_error_plot(self):
        all_setpoints = sorted(self.error_plot_data.keys())
        if not all_setpoints:
            return
            
        self._setup_error_plot(all_setpoints)
        
        bar_height = 1.0
        num_duts = len(self.active_duts)
        
        if len(all_setpoints) > 1:
            min_gap = min(np.diff(all_setpoints))
            bar_height = min(min_gap * 0.8, 2.0)
        
        bar_height /= num_duts
        
        for i, dut in enumerate(self.active_duts):
            ch = dut['channel']
            offset = (i - (num_duts - 1) / 2) * bar_height
            
            for sp, errors_data in self.error_plot_data.items():
                if ch in errors_data:
                    error_info = errors_data[ch]
                    error_val = error_info['error']
                    log_time = error_info['time']

                    self.ax_error.barh(sp + offset, error_val, height=bar_height,
                                       color=self.dut_colors[ch], edgecolor='black', linewidth=0.5)
                    
                    ha = 'left' if error_val >= 0 else 'right'
                    self.ax_error.text(error_val, sp + offset, f" @{log_time:.1f}s",
                                       va='center', ha=ha, fontsize=7, color='#555555')
        
        self.ax_error.relim()
        self.ax_error.autoscale_view(scalex=True, scaley=False)
        self.canvas.draw_idle()

    def on_closing(self):
        self.is_calibrating = False; self.is_in_manual_mode = False
        self._save_learned_data()
        
        if self.after_id:
            self.after_cancel(self.after_id)
            self.after_id = None

        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()
        
        self.quit()
        self.destroy()

if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()