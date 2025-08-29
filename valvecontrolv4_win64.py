# -*- coding: utf-8 -*-
# ==============================================================================
# Script Name: Multi-Device Integrated Calibration Script
# Author: Gemini
# Date: August 20, 2025
# Description: This script provides a full calibration workflow, including a
#              manual calibration stage with live feedback for zero, span, and
#              linearity adjustments, followed by an automated data logging run
#              with intelligent tuning suggestions.
#
# Version 11 Changes:
#   - Redesigned Manual Calibration screen with a two-panel layout.
#   - Added a large, high-resolution live trace plot to the manual screen that
#     appears when a specific DUT is selected for focus.
#   - The new plot shows the focused DUT vs. the Standard in real-time.
#   - Implemented an auto-scaling Y-axis on the manual plot for a magnified
#     view of fine adjustments.
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
        """Commands the DAQ to read a specific channel and returns a smoothed value."""
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
# BaratronController Class ("Standard")
# =================================================================================
class BaratronController:
    """Handles communication with the MKS 651 Pressure Controller."""
    RANGE_CODES = {
        0.1: 0, 0.2: 1, 0.5: 2, 1.0: 3, 2.0: 4, 5.0: 5, 10.0: 6, 20.0: 20,
        50.0: 7, 100.0: 8, 500.0: 9, 1000.0: 10, 5000.0: 11, 10000.0: 12, 1.33: 13,
        2.66: 14, 13.33: 15, 133.3: 16, 1333.0: 17, 6666.0: 18, 13332.0: 19
    }

    def __init__(self, port, full_scale_pressure):
        try:
            self.ser = serial.Serial(
                port=port, baudrate=9600, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2
            )
            self.full_scale_pressure = full_scale_pressure
            self.is_connected = True
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to open Standard port {port}: {e}")

    def write_command(self, command):
        if not self.is_connected: return
        full_command = (command + '\r').encode('ascii')
        self.ser.write(full_command)

    def query_command(self, command):
        if not self.is_connected: return None
        self.write_command(command)
        response_bytes = self.ser.readline()
        return response_bytes.decode('ascii', errors='ignore').strip()

    def set_pressure(self, pressure):
        setpoint_percentage = (pressure / self.full_scale_pressure) * 100
        setpoint_command = f"S1 {setpoint_percentage:.2f}"
        self.write_command(setpoint_command)
        self.write_command("D1")
        
    def get_pressure(self):
        response = self.query_command("R5")
        if response:
            try:
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    percentage_reading = float(match.group())
                    return (percentage_reading / 100) * self.full_scale_pressure
            except (ValueError, IndexError): return None
        return None
    
    def get_valve_position(self):
        response = self.query_command("R6")
        if response:
            try:
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    return float(match.group())
            except (ValueError, IndexError): return None
        return None
        
    def close_valve(self):
        self.write_command("C")
            
    def close(self):
        if self.is_connected and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
        
# =================================================================================
# Main GUI Class
# =================================================================================
class CalibrationGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.title("Multi-Device Baratron Calibration System")
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        self.baratron = None
        self.daq = None
        self.is_calibrating = False
        self.is_in_manual_mode = False
        
        self.data_storage = {}
        self.error_tracker = {}
        self.log_queue = queue.Queue()
        self.manual_data_queue = queue.Queue()
        self.manual_command_queue = queue.Queue()
        self.live_data_queue = queue.Queue()
        self.manual_trace_queue = queue.Queue() # Queue for the new manual plot
        self.manual_focus_device = tk.StringVar(value="std")
        self.manual_focus_channel = None

        self.live_time_history = []
        self.live_valve_history = []
        self.live_std_pressure_history = []
        self.live_dut_pressure_history = {i: [] for i in range(4)}

        # Data history for the new manual trace plot
        self.manual_trace_time = []
        self.manual_trace_std = []
        self.manual_trace_dut = []
        
        self.setup_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after(100, self.periodic_update)

    def setup_ui(self):
        # ... (This method is unchanged)
        top_config_frame = tk.Frame(self)
        top_config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        top_config_frame.columnconfigure(0, weight=1); top_config_frame.columnconfigure(1, weight=1)

        config_frame = tk.LabelFrame(top_config_frame, text="Configuration", padx=10, pady=10)
        config_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        valid_ranges = sorted(BaratronController.RANGE_CODES.keys())

        tk.Label(config_frame, text="Standard Controller:").grid(row=0, column=0, sticky="w", columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=1, column=0, sticky="e", padx=5)
        self.std_com_var = tk.StringVar(self, value="COM9")
        self.std_com_combo = ttk.Combobox(config_frame, textvariable=self.std_com_var, values=com_ports, width=10)
        self.std_com_combo.grid(row=1, column=1, sticky="w")
        tk.Label(config_frame, text="FS (Torr):").grid(row=2, column=0, sticky="e", padx=5)
        self.std_fs_var = tk.StringVar(self); self.std_fs_var.set(100.0)
        self.std_fs_menu = tk.OptionMenu(config_frame, self.std_fs_var, *valid_ranges)
        self.std_fs_menu.grid(row=2, column=1, sticky="w")

        tk.Label(config_frame, text="DAQ (RP2040):").grid(row=3, column=0, sticky="w", pady=(10,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=4, column=0, sticky="e", padx=5)
        self.daq_com_var = tk.StringVar(self, value="COM12")
        self.daq_com_combo = ttk.Combobox(config_frame, textvariable=self.daq_com_var, values=com_ports, width=10)
        self.daq_com_combo.grid(row=4, column=1, sticky="w")

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

        self.fig, (self.ax_cal, self.ax_live) = plt.subplots(2, 1, figsize=(12, 10)); self.fig.subplots_adjust(hspace=0.6)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_term_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        term_frame = tk.Frame(self.plot_term_frame)
        term_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        term_frame.columnconfigure(0, weight=1)
        
        self.terminal_text = scrolledtext.ScrolledText(term_frame, height=10, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00")
        self.terminal_text.pack(fill="both", expand=True)
        
        input_frame = tk.Frame(term_frame)
        input_frame.pack(fill=tk.X, pady=(5, 0))
        tk.Label(input_frame, text="Standard CMD:").pack(side=tk.LEFT)
        self.command_entry = tk.Entry(input_frame, font=("Courier", 10), bg="#2c2c2c", fg="#00ff00", insertbackground="#00ff00")
        self.command_entry.pack(fill=tk.X, expand=True, side=tk.LEFT)
        self.command_entry.bind("<Return>", self.send_baratron_command)

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

    def send_baratron_command(self, event=None):
        command = self.command_entry.get().strip()
        self.command_entry.delete(0, tk.END)
        if not command: return
        self.log_message(f"> {command}")
        if self.baratron and self.baratron.is_connected:
            try:
                response = self.baratron.query_command(command)
                if response: self.log_message(f"Response: {response}")
                else: self.log_message("Command sent (no response).")
            except Exception as e: self.log_message(f"Error sending command: {e}")
        else: self.log_message("Standard not connected.")
        
    def connect_instruments(self):
        try:
            self.standard_fs_value = float(self.std_fs_var.get())
            self.baratron = BaratronController(self.std_com_var.get(), full_scale_pressure=self.standard_fs_value)
            self.log_message(f"Connected to Standard on {self.std_com_var.get()}.")
            
            self.daq = DAQController(self.daq_com_var.get())
            self.log_message(f"Connected to DAQ on {self.daq_com_var.get()}.")

            self.active_duts = [{'channel': i, 'fs': float(w['fs'].get())} for i, w in enumerate(self.dut_widgets) if w['enabled'].get()]
            if not self.active_duts: raise ValueError("At least one Device Under Test must be enabled.")

            self.start_button.config(state=tk.NORMAL); self.e_stop_button.config(state=tk.DISABLED)
            self.manual_cal_button.config(state=tk.NORMAL)
            self.connect_button.config(state=tk.DISABLED)
            self.set_config_state(tk.DISABLED)
            self.configure_plots()
        except (ValueError, ConnectionError) as e:
            self.log_message(f"ERROR: {e}")

    def set_config_state(self, state):
        # ... (This method is unchanged)
        self.std_com_combo.config(state=state); self.std_fs_menu.config(state=state)
        self.daq_com_combo.config(state=state)
        for widget_set in self.dut_widgets:
            widget_set['check'].config(state=state); widget_set['menu'].config(state=state)

    def e_stop_action(self):
        # ... (This method is unchanged)
        if self.is_calibrating or self.is_in_manual_mode:
            self.is_calibrating = False; self.is_in_manual_mode = False
            self.log_message("\n*** E-STOP ***\nProcess stopped.")
            if self.baratron: self.baratron.close_valve()
            self.start_button.config(state=tk.NORMAL); self.manual_cal_button.config(state=tk.NORMAL)
            self.manual_cal_button.config(text="Manual Cal")
            self.e_stop_button.config(state=tk.DISABLED)
        
    def configure_plots(self):
        # ... (This method is unchanged)
        self.ax_cal.clear()
        self.ax_live.clear()
        if hasattr(self, 'ax_live_pressure'): self.ax_live_pressure.clear()

        self.ax_cal.set_title("Calibration Curve: Standard vs. Devices")
        self.ax_cal.set_xlabel("Standard Pressure (Torr)"); self.ax_cal.set_ylabel("Device Pressure (Torr)")
        self.ax_cal.grid(True)
        max_fs = self.standard_fs_value
        for dut in self.active_duts: max_fs = max(max_fs, dut['fs'])
        self.ax_cal.set_xlim([0, max_fs*1.05]); self.ax_cal.set_ylim([0, max_fs*1.05])
        self.ax_cal.plot([0, max_fs], [0, max_fs], 'k--', alpha=0.5, label='Ideal 1:1 Line')
        
        self.ax_live.set_title("Live Vitals"); self.ax_live.set_xlabel("Time (s)")
        self.ax_live.grid(True, linestyle=':')
        
        self.ax_live.set_ylabel("Valve Position (%)", color='darkorange')
        self.ax_live.tick_params(axis='y', labelcolor='darkorange')
        self.ax_live.set_ylim([-5, 105])
        self.live_valve_plot, = self.ax_live.plot([], [], color='darkorange', linestyle='--', label='Valve Pos')

        self.ax_live_pressure = self.ax_live.twinx()
        self.ax_live_pressure.set_ylabel("Pressure (Torr)", color='royalblue')
        self.ax_live_pressure.tick_params(axis='y', labelcolor='royalblue')
        self.ax_live_pressure.set_ylim([-0.05*self.standard_fs_value, self.standard_fs_value * 1.05])
        
        self.live_std_plot, = self.ax_live_pressure.plot([], [], 'blue', linewidth=2, label='Standard')
        dut_colors = ['green', 'red', 'purple', 'brown']
        self.live_dut_plots = {}
        for i in range(4):
            line, = self.ax_live_pressure.plot([], [], color=dut_colors[i], label=f'DUT {i+1}')
            self.live_dut_plots[i] = line
        
        lines, labels = self.ax_live.get_legend_handles_labels()
        lines2, labels2 = self.ax_live_pressure.get_legend_handles_labels()
        self.ax_live_pressure.legend(lines + lines2, labels + labels2, loc='upper left')

        self.canvas.draw_idle()

    def log_message(self, message):
        self.log_queue.put(message)

    def periodic_update(self):
        # ... (Code added to handle the new manual trace queue)
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.terminal_text.config(state=tk.NORMAL); self.terminal_text.insert(tk.END, f"\n{msg}"); self.terminal_text.see(tk.END); self.terminal_text.config(state=tk.DISABLED)
        
        if self.is_in_manual_mode:
            try:
                data = self.manual_data_queue.get_nowait()
                self.update_manual_display(data)
            except queue.Empty:
                pass
            
            # --- NEW: Update the manual trace plot if it's active ---
            trace_data_processed = False
            while not self.manual_trace_queue.empty():
                trace_data_processed = True
                data = self.manual_trace_queue.get()
                self.manual_trace_time.append(data['time'])
                self.manual_trace_std.append(data['std'])
                self.manual_trace_dut.append(data['dut'])
            
            if trace_data_processed:
                # Keep the trace to a 20-second rolling window
                if len(self.manual_trace_time) > 100: # 100 samples at 200ms = 20s
                    self.manual_trace_time.pop(0)
                    self.manual_trace_std.pop(0)
                    self.manual_trace_dut.pop(0)
                
                self.manual_std_line.set_data(self.manual_trace_time, self.manual_trace_std)
                self.manual_dut_line.set_data(self.manual_trace_time, self.manual_trace_dut)
                
                t_max = self.manual_trace_time[-1] if self.manual_trace_time else 0
                self.ax_manual_trace.set_xlim(max(0, t_max - 20), t_max + 1)
                self.ax_manual_trace.relim()
                self.ax_manual_trace.autoscale_view(scaley=True) # Auto-scale Y-axis
                self.manual_canvas.draw_idle()

        processed_data = False
        while not self.live_data_queue.empty():
            # ... (rest of the function is unchanged)
            processed_data = True
            data = self.live_data_queue.get()
            
            self.live_time_history.append(data['time'])
            self.live_valve_history.append(data['valve'])
            self.live_std_pressure_history.append(data['std'])
            
            active_dut_channels = data['duts'].keys()
            for i in range(4):
                if i in active_dut_channels:
                    self.live_dut_pressure_history[i].append(data['duts'][i])
                else:
                    self.live_dut_pressure_history[i].append(np.nan)

        if processed_data:
            if len(self.live_time_history) > 450:
                self.live_time_history.pop(0)
                self.live_valve_history.pop(0)
                self.live_std_pressure_history.pop(0)
                for i in range(4): self.live_dut_pressure_history[i].pop(0)

            self.live_valve_plot.set_data(self.live_time_history, self.live_valve_history)
            self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
            
            for i, line in self.live_dut_plots.items():
                is_active = any(d['channel'] == i for d in self.active_duts)
                line.set_visible(is_active)
                if is_active:
                    line.set_data(self.live_time_history, self.live_dut_pressure_history[i])
            
            t_max = self.live_time_history[-1] if self.live_time_history else 0
            self.ax_live.set_xlim(max(0, t_max - 90), t_max + 1)
            self.ax_live.relim(); self.ax_live.autoscale_view(scalex=False, scaley=True)
            self.ax_live_pressure.relim(); self.ax_live_pressure.autoscale_view(scalex=False, scaley=True)
            self.canvas.draw_idle()
        
        self.after(100, self.periodic_update)

    def clear_live_data(self):
        # ... (This method is unchanged)
        self.live_time_history.clear()
        self.live_valve_history.clear()
        self.live_std_pressure_history.clear()
        for i in range(4):
            self.live_dut_pressure_history[i].clear()
        while not self.live_data_queue.empty():
            self.live_data_queue.get()

    def toggle_manual_mode(self):
        # ... (This method is unchanged)
        self.is_in_manual_mode = not self.is_in_manual_mode
        if self.is_in_manual_mode:
            self.clear_live_data()
            self.manual_cal_button.config(text="Exit Manual Cal")
            self.start_button.config(state=tk.DISABLED)
            self.e_stop_button.config(state=tk.NORMAL)
            self.setup_manual_display()
            threading.Thread(target=self.run_manual_loop, daemon=True).start()
        else:
            self.manual_cal_button.config(text="Manual Cal")
            self.start_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.DISABLED)
            self.teardown_manual_display()
            if self.baratron: self.baratron.close_valve()

    def setup_manual_display(self):
        # --- ENTIRE METHOD REWRITTEN ---
        self.canvas.get_tk_widget().grid_remove() # Hide the main plots
        self.manual_frame = tk.Frame(self.plot_term_frame)
        self.manual_frame.grid(row=0, column=0, sticky="nsew")

        # Configure a two-panel layout
        self.manual_frame.rowconfigure(0, weight=1)
        self.manual_frame.columnconfigure(0, weight=1) # Left panel for controls
        self.manual_frame.columnconfigure(1, weight=3) # Right panel for new plot

        # --- Left Panel (Controls) ---
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

        # --- Right Panel (Live Trace Plot) ---
        self.manual_plot_panel = tk.Frame(self.manual_frame)
        self.manual_plot_panel.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        self.manual_fig, self.ax_manual_trace = plt.subplots(figsize=(8, 6))
        self.ax_manual_trace.set_title("Live Trace: DUT vs. Standard")
        self.ax_manual_trace.set_xlabel("Time (s)")
        self.ax_manual_trace.set_ylabel("Pressure (Torr)")
        self.ax_manual_trace.grid(True)
        self.manual_std_line, = self.ax_manual_trace.plot([], [], 'b-', label='Standard')
        self.manual_dut_line, = self.ax_manual_trace.plot([], [], 'g-', label='Focused DUT', linewidth=2)
        self.ax_manual_trace.legend()
        
        self.manual_canvas = FigureCanvasTkAgg(self.manual_fig, master=self.manual_plot_panel)
        self.manual_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.on_manual_focus_change() # Initial setup of visibility

    def on_manual_focus_change(self):
        # --- REWRITTEN to control plot visibility ---
        focus_id = self.manual_focus_device.get()
        if focus_id == 'std':
            self.manual_focus_channel = None
            self.log_message(f"Manual control focus set to Standard.")
            if hasattr(self, 'manual_plot_panel'):
                self.manual_plot_panel.grid_remove() # Hide plot
        else:
            ch = int(focus_id.replace('ch',''))
            self.manual_focus_channel = ch
            fs = [d['fs'] for d in self.active_duts if d['channel'] == ch][0]
            self.log_message(f"Manual control focus set to DUT {ch+1} ({fs} Torr).")
            
            # Clear old data and show the plot
            self.manual_trace_time.clear()
            self.manual_trace_std.clear()
            self.manual_trace_dut.clear()
            if hasattr(self, 'manual_plot_panel'):
                self.ax_manual_trace.set_title(f"Live Trace: DUT {ch+1} vs. Standard")
                self.manual_plot_panel.grid() # Show plot

    def set_manual_pressure(self, fs_fraction):
        # ... (This method is unchanged)
        focus_id = self.manual_focus_device.get()
        target_fs = 0

        if focus_id == "std":
            target_fs = self.standard_fs_value
        else:
            ch = int(focus_id.replace('ch',''))
            for dut in self.active_duts:
                if dut['channel'] == ch:
                    target_fs = dut['fs']
                    break
        
        if target_fs > 0 or fs_fraction == 0:
            pressure = target_fs * fs_fraction
            self.manual_command_queue.put({'command': 'set_pressure', 'value': pressure})
            self.manual_setpoint_var.set(f"Current Setpoint: {pressure:.2f} Torr")
        else:
            self.log_message("Error: Could not find focus device to set pressure.")

    def update_manual_display(self, data):
        # ... (This method is unchanged)
        for ch, values in data.items():
            diff = values['diff']
            self.manual_labels[ch]['diff_var'].set(f"Diff: {diff:+.3f} Torr")
            
            inner_tolerance = values['tolerance'] 
            outer_tolerance = inner_tolerance * 5 

            fill_percent = 0.0
            if abs(diff) <= inner_tolerance:
                fill_percent = 100.0
            elif abs(diff) < outer_tolerance:
                progress = (abs(diff) - inner_tolerance) / (outer_tolerance - inner_tolerance)
                fill_percent = 100.0 * (1.0 - progress)
            
            self.manual_labels[ch]['progress']['value'] = fill_percent

            style_name = f"ch{ch}.Horizontal.TProgressbar"
            s = ttk.Style()
            s.configure(style_name, troughcolor='#e0e0e0', background='green')
            self.manual_labels[ch]['progress'].config(style=style_name)

    def teardown_manual_display(self):
        if hasattr(self, 'manual_frame'): self.manual_frame.destroy()
        self.canvas.get_tk_widget().grid()

    def run_manual_loop(self):
        # ... (Code added to put data onto the new manual trace queue)
        self.log_message("Entering Manual Calibration Mode...")
        start_time = time.time()
        current_setpoint = 0.0
        self.manual_command_queue.put({'command': 'set_pressure', 'value': current_setpoint})
        
        while self.is_in_manual_mode:
            try:
                cmd = self.manual_command_queue.get_nowait()
                if cmd['command'] == 'set_pressure':
                    current_setpoint = cmd['value']
                    self.log_message(f"Manual Mode: Setting pressure to {current_setpoint:.2f} Torr")
                    if self.baratron: self.baratron.set_pressure(current_setpoint)
            except queue.Empty:
                pass
            
            std_pressure = self.baratron.get_pressure() if self.baratron else None
            valve_pos = self.baratron.get_valve_position() if self.baratron else None
            
            manual_packet = {}
            live_dut_pressures = {}
            for dut in self.active_duts:
                ch, fs = dut['channel'], dut['fs']
                voltage = self.daq.read_voltage(ch) if self.daq else None
                
                if voltage is not None:
                    device_pressure = voltage * (fs / 10.0)
                    live_dut_pressures[ch] = device_pressure
                    if std_pressure is not None:
                        diff = device_pressure - std_pressure
                        tolerance = fs * 0.002
                        manual_packet[ch] = {'diff': diff, 'tolerance': tolerance}
            
            if manual_packet: self.manual_data_queue.put(manual_packet)
            
            # --- NEW: Send data to the manual trace plot if a DUT is focused ---
            if self.manual_focus_channel is not None and self.manual_focus_channel in live_dut_pressures:
                trace_packet = {
                    'time': time.time() - start_time,
                    'std': std_pressure,
                    'dut': live_dut_pressures[self.manual_focus_channel]
                }
                self.manual_trace_queue.put(trace_packet)
            
            # This live_data_queue is for the small, top-right plot in the main view,
            # which is currently hidden but we can feed it data anyway.
            live_data_packet = {
                'time': time.time() - start_time, 'valve': valve_pos, 
                'std': std_pressure, 'duts': live_dut_pressures
            }
            self.live_data_queue.put(live_data_packet)

            time.sleep(0.2)
        self.log_message("Exited Manual Calibration Mode.")
    
    # ... (The rest of the script, including run_calibration, analyze_and_suggest_tuning, etc., is unchanged)
    def start_calibration_thread(self):
        self.clear_live_data()
        self.is_calibrating = True
        self.log_message("\n--- Starting Automated Data Logging ---")
        self.data_storage = {'Setpoint_Torr': [], 'Standard_Pressure_Torr': []}
        for dut in self.active_duts:
            self.data_storage[f'Device_{dut["channel"]+1}_Pressure_Torr'] = []
        
        self.start_button.config(state=tk.DISABLED); self.manual_cal_button.config(state=tk.DISABLED)
        self.e_stop_button.config(state=tk.NORMAL)
        threading.Thread(target=self.run_calibration, daemon=True).start()

    def run_calibration(self):
        start_time = time.time()
        self.error_tracker = {dut['channel']: [] for dut in self.active_duts}
        try:
            master_setpoints = set()
            for i in range(0, 101, 10):
                master_setpoints.add(round(self.standard_fs_value * i / 100, 2))
            for dut in self.active_duts:
                for i in range(0, 101, 10):
                    master_setpoints.add(round(dut['fs'] * i / 100, 2))
            
            setpoints = sorted(list(master_setpoints))
            self.log_message(f"Generated composite setpoints: {setpoints}")

            dut_specific_setpoints = {
                dut['channel']: {round(dut['fs'] * i / 100, 2) for i in range(0, 101, 10)}
                for dut in self.active_duts
            }
            
            for sp in setpoints:
                if not self.is_calibrating: break
                self.log_message(f"\n--- Setting {sp} Torr ---")
                self.baratron.set_pressure(sp)
                
                self.log_message("Waiting for pressure to stabilize within tolerance...")
                pressure_history = collections.deque(maxlen=20)
                stability_confirmed_time = None
                last_live_poll_time = time.time()
                notified_out_of_tolerance = False
                zero_fail_counter = 0

                relevant_duts = [d for d in self.active_duts if sp in dut_specific_setpoints[d['channel']]]
                if not relevant_duts:
                    priority_tolerance = self.standard_fs_value * 0.005 
                else:
                    tolerances = [dut['fs'] * 0.005 for dut in relevant_duts]
                    priority_tolerance = min(tolerances)

                while self.is_calibrating:
                    if (time.time() - last_live_poll_time) > 0.2:
                        pressure = self.baratron.get_pressure()
                        valve_pos = self.baratron.get_valve_position()
                        if pressure is not None: pressure_history.append(pressure)
                        
                        live_dut_pressures = {
                            dut['channel']: self.daq.read_voltage(dut['channel']) * (dut['fs'] / 10.0)
                            for dut in self.active_duts
                        }
                        live_data_packet = {'time': time.time() - start_time, 'valve': valve_pos, 'std': pressure, 'duts': live_dut_pressures}
                        self.live_data_queue.put(live_data_packet)
                        last_live_poll_time = time.time()

                    if len(pressure_history) == pressure_history.maxlen:
                        is_stable = np.std(pressure_history) < (self.standard_fs_value * 0.0002)
                        
                        if is_stable:
                            stable_pressure = np.mean(pressure_history)
                            is_in_tolerance = abs(stable_pressure - sp) <= priority_tolerance

                            if is_in_tolerance:
                                if notified_out_of_tolerance:
                                    self.log_message("  Pressure is now stable within the tolerance window.")
                                    notified_out_of_tolerance = False
                                
                                if stability_confirmed_time is None: stability_confirmed_time = time.time()
                                if (time.time() - stability_confirmed_time) >= 2.0:
                                    self.log_message(f"  Pressure locked at {stable_pressure:.3f} Torr. Proceeding to log.")
                                    break
                            else:
                                stability_confirmed_time = None
                                if not notified_out_of_tolerance:
                                    self.log_message(f"  Pressure stable at {stable_pressure:.3f} Torr, but OUTSIDE tolerance window (+/- {priority_tolerance:.4f} Torr). Waiting...")
                                    notified_out_of_tolerance = True
                                    if sp == 0:
                                        zero_fail_counter += 1
                                        if zero_fail_counter >= 3:
                                            self.log_message("  Zero point failed to stabilize 3 times. Prompting user...")
                                            should_proceed = messagebox.askyesno("Zero Point Override", 
                                                f"The pressure is stable at {stable_pressure:.4f} Torr, but this is outside the tolerance for the 0 Torr setpoint.\n\n"
                                                "Do you want to accept this reading and proceed?")
                                            
                                            if should_proceed:
                                                self.log_message("  User accepted the out-of-tolerance zero reading.")
                                                break
                                            else:
                                                self.log_message("  User chose to continue waiting. Resetting failure count.")
                                                zero_fail_counter = 0
                        else:
                            stability_confirmed_time = None
                            if notified_out_of_tolerance:
                                self.log_message("  Pressure is no longer stable. Resuming...")
                                notified_out_of_tolerance = False
                    
                    time.sleep(0.05)
                
                if not self.is_calibrating: continue

                self.log_message(f"  Starting 10s data log.")
                log_start_time = time.time()
                standard_readings = []
                dut_readings = {dut['channel']: [] for dut in self.active_duts}
                
                while (time.time() - log_start_time) < 10.0 and self.is_calibrating:
                    s_press = self.baratron.get_pressure()
                    if s_press is not None: standard_readings.append(s_press)
                    
                    live_dut_pressures = {}
                    for dut in self.active_duts:
                        ch, fs = dut['channel'], dut['fs']
                        voltage = self.daq.read_voltage(ch)
                        if voltage is not None:
                            device_pressure = voltage * (fs / 10.0)
                            dut_readings[ch].append(device_pressure)
                            live_dut_pressures[ch] = device_pressure
                    
                    valve_pos = self.baratron.get_valve_position()
                    live_data_packet = {'time': time.time() - start_time, 'valve': valve_pos, 'std': s_press, 'duts': live_dut_pressures}
                    self.live_data_queue.put(live_data_packet)
                    time.sleep(0.2)

                if not self.is_calibrating: continue

                mean_standard = np.mean(standard_readings) if standard_readings else np.nan
                if np.isnan(mean_standard):
                    self.log_message(f"ERROR: Failed to read Standard for setpoint {sp}."); continue

                self.data_storage['Setpoint_Torr'].append(sp)
                self.data_storage['Standard_Pressure_Torr'].append(mean_standard)
                log_line = f"  Logged -> Setpoint: {sp:.2f} | Standard (Avg): {mean_standard:.3f} Torr"

                for dut in self.active_duts:
                    ch, fs = dut['channel'], dut['fs']
                    
                    if sp in dut_specific_setpoints[ch]:
                        mean_dut = np.mean(dut_readings.get(ch, [])) if dut_readings.get(ch) else np.nan
                        self.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(mean_dut)
                        if not np.isnan(mean_dut):
                            log_line += f" | Dev {ch+1} (Avg): {mean_dut:.3f} Torr"
                            error = mean_dut - mean_standard
                            tolerance = fs * 0.005
                            if abs(error) > tolerance:
                                self.error_tracker[ch].append({'std': mean_standard, 'dut': mean_dut, 'error': error})
                        else:
                            log_line += f" | Dev {ch+1}: READ FAILED"
                    else:
                        self.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(np.nan)

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
            if self.baratron: self.baratron.close_valve()
            self.after(10, self.analyze_and_suggest_tuning)
            self.after(10, lambda: [
                self.start_button.config(state=tk.NORMAL),
                self.manual_cal_button.config(state=tk.NORMAL),
                self.e_stop_button.config(state=tk.DISABLED)
            ])

    def show_tuning_suggestions_window(self, suggestions_text):
        win = tk.Toplevel(self)
        win.title("Tuning Suggestions")
        win.geometry("700x600")

        main_frame = tk.Frame(win, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        suggestion_box = scrolledtext.ScrolledText(
            main_frame, height=10, font=("Courier", 10), 
            wrap=tk.WORD, relief=tk.SOLID, borderwidth=1
        )
        suggestion_box.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        suggestion_box.insert(tk.END, suggestions_text)
        suggestion_box.config(state=tk.DISABLED)

        close_button = tk.Button(main_frame, text="Close", command=win.destroy)
        close_button.pack()

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

            mid_pressure_target = fs * 0.5
            mid_idx = (np.abs(std_points - mid_pressure_target)).argmin()
            midpoint_error_from_line = dut_points[mid_idx] - (slope * std_points[mid_idx] + intercept)
            linearity_is_sig = abs(midpoint_error_from_line) > (fs * 0.002)

            if not (zero_offset_is_sig or span_error_is_sig or linearity_is_sig):
                suggestion_parts.append(f"\n--- Analysis for DUT {ch+1} ({fs} Torr FS) ---")
                suggestion_parts.append("  ✅ SUCCESS: Device is well-calibrated. No significant Zero, Span, or Linearity errors detected.")
                continue

            any_suggestions = True
            suggestion_parts.append(f"\n--- Suggestions for DUT {ch+1} ({fs} Torr FS) ---")
            
            suggestion_parts.append("\n[ DIAGNOSIS ]")
            suggestion_parts.append(f"The device's response fits the line: y = {slope:.4f}x + {intercept:+.4f}")
            if zero_offset_is_sig:
                suggestion_parts.append(f" • ZERO OFFSET ERROR: The device has an offset of {intercept:+.4f} Torr.")
            if span_error_is_sig:
                suggestion_parts.append(f" • SPAN (GAIN) ERROR: The device's gain is {'too high' if slope > 1 else 'too low'} (Slope={slope:.4f}).")
            if linearity_is_sig:
                suggestion_parts.append(f" • LINEARITY ERROR: The mid-range response {'bows UP' if midpoint_error_from_line > 0 else 'bows DOWN'}.")

            suggestion_parts.append("\n[ RECOMMENDED ADJUSTMENT PLAN ]")
            suggestion_parts.append("Follow these steps in order for best results:")

            suggestion_parts.append("\n1. ADJUST ZERO (0% FS)")
            suggestion_parts.append("   Apply ZERO pressure (0 Torr). Adjust the ZERO potentiometer.")
            if zero_offset_is_sig:
                direction = "LOWER" if intercept > 0 else "RAISE"
                suggestion_parts.append(f"   ➡️ ACTION: Your device reads high at zero. Adjust to {direction} the reading to match the standard.")
                if abs(intercept) > (fs * 0.02):
                    suggestion_parts.append("   ⚠️ NOTE: This is a large offset. Use the COARSE ZERO pot first if available.")
            else:
                suggestion_parts.append("   - Your zero offset is minor. Confirm it is correct.")

            suggestion_parts.append("\n2. ADJUST SPAN (100% FS)")
            suggestion_parts.append("   Apply FULL SCALE pressure (100% FS). Adjust the SPAN potentiometer.")
            if span_error_is_sig:
                direction = "LOWER" if slope > 1 else "RAISE"
                suggestion_parts.append(f"   ➡️ ACTION: Your device's gain is too {'high' if slope > 1 else 'low'}. Adjust to {direction} the reading to match the standard at 100% FS.")
            else:
                suggestion_parts.append("   - Your span/gain is close to correct. Confirm and make minor adjustments if needed.")
            
            suggestion_parts.append("\n3. RE-CHECK ZERO (Critical Step)")
            suggestion_parts.append("   Return to ZERO pressure. The Span adjustment likely affected the Zero setting.")
            suggestion_parts.append("   ➡️ ACTION: Re-adjust the ZERO pot. You may need to repeat steps 1 and 2 a few times.")

            suggestion_parts.append("\n4. ADJUST LINEARITY (50% FS)")
            suggestion_parts.append("   Apply MID-RANGE pressure (50% FS). Adjust the LINEARITY potentiometer.")
            if linearity_is_sig:
                suggestion_parts.append(f"   ➡️ ACTION: Your device's response {'bows upward (\"smiling\")' if midpoint_error_from_line > 0 else 'bows downward (\"frowning\")'}.")
                suggestion_parts.append("      Make small adjustments to the Linearity pot to correct this midpoint error.")
            else:
                suggestion_parts.append("   - Your linearity appears to be good. Confirm and make minor adjustments if needed.")
            suggestion_parts.append("   (Note: Adjusting Linearity can slightly affect Zero and Span. A final check is recommended.)")

        if not any_suggestions and any(len(self.error_tracker.get(d['channel'], [])) > 0 for d in self.active_duts):
             suggestion_parts.append("\nSome devices had minor errors but none were significant enough to suggest a specific adjustment.")
        
        final_suggestion_text = "\n".join(suggestion_parts)
        self.log_message(final_suggestion_text)
        
        if any_suggestions:
            self.show_tuning_suggestions_window(final_suggestion_text)

    def update_cal_plot(self):
        self.ax_cal.clear()
        self.ax_cal.set_title("Calibration Curve: Standard vs. Devices")
        self.ax_cal.set_xlabel("Standard Pressure (Torr)"); self.ax_cal.set_ylabel("Device Pressure (Torr)")
        self.ax_cal.grid(True)
        max_fs = self.standard_fs_value
        for dut in self.active_duts: max_fs = max(max_fs, dut['fs'])
        self.ax_cal.set_xlim([0, max_fs*1.05]); self.ax_cal.set_ylim([0, max_fs*1.05])
        self.ax_cal.plot([0, max_fs], [0, max_fs], 'k--', alpha=0.5, label='Ideal 1:1 Line')
        
        std_data = self.data_storage['Standard_Pressure_Torr']
        for dut in self.active_duts:
            dut_data = self.data_storage[f'Device_{dut["channel"]+1}_Pressure_Torr']
            valid_points = [(s, d) for s, d in zip(std_data, dut_data) if not (np.isnan(s) or np.isnan(d))]
            if valid_points:
                std_plot, dut_plot = zip(*valid_points)
                self.ax_cal.plot(std_plot, dut_plot, 'o-', label=f'Device {dut["channel"]+1}')

        self.ax_cal.legend()
        self.canvas.draw_idle()

    def on_closing(self):
        self.is_calibrating = False
        self.is_in_manual_mode = False
        time.sleep(0.3)
        if self.baratron: self.baratron.close()
        if self.daq: self.daq.close()
        self.destroy()

# =================================================================================
# Main Script Execution
# =================================================================================
if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()