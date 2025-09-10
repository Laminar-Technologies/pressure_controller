# -*- coding: utf-8 -*-
# ==============================================================================
# File:         Main.py
# Author:       Gemini
# Date:         September 5, 2025
# Description:  This is the main application file for the Multi-Device
#               Adaptive Calibration System. It initializes the GUI,
#               manages device connections, runs the automated calibration
#               sequence, and orchestrates the other modules.
#
# Version 118 (Initialization Bug Fix):
#   - Corrected startup crash by initializing valve plot axes (ax_inlet_valve,
#     ax_outlet_valve) to None in the __init__ method, preventing a race
#     condition during UI setup.
#   - Restored missing turbo pump status indicator LEDs.
#   - Implemented E-Stop recovery via a "Resume" button.
#   - Correctly positioned all valve adjustment buttons.
# ==============================================================================

# --- Standard Library Imports ---
import time
import queue
import json
import threading
import collections
import statistics

# --- Third-Party Imports ---
import pandas as pd
import numpy as np
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import scrolledtext, ttk, messagebox

# --- Local Application Imports ---
from DAQ_Controller import DAQController
from State_Machine_Controller import StateMachinePressureController
from Manual_Cal import ManualCalFrame
from Tuning_Analysis import generate_tuning_suggestions
from Auto_Cal_Logic import run_calibration
from Turbo_Controller import TurboController

# =================================================================================
# Main GUI Class
# =================================================================================
class CalibrationGUI(tk.Tk):
    """
    The main application window for the calibration system.
    """
    def __init__(self):
        super().__init__()

        self.title("Multi-Device Adaptive Calibration System")
        self.state('zoomed')
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        # --- Initialize Controllers and State ---
        self.state_controller = None
        self.daq = None
        self.turbo_controller = None
        self.is_calibrating = False
        self.is_in_manual_mode = False
        self.start_time = 0
        self.after_id = None
        self.manual_frame = None
        self.e_stop_triggered_event = threading.Event()

        self.turbo_rpm_var = tk.StringVar(value="---- RPM")
        self.turbo_temp_var = tk.StringVar(value="--°C")
        self.dut_colors = ['#2ca02c', '#d62728', '#ffd700', '#8c564b']
        
        # --- Color Palette for Turbine and LEDs ---
        self.at_speed_color = '#00E676'
        self.starting_color = '#FFD600'
        self.stopped_color = '#a0a0a0'

        self.data_storage = {}
        self.error_plot_data = {}
        self.log_queue = queue.Queue()
        
        self.completed_duts = set()

        # --- Learned Positions Management ---
        self.learned_positions_file = "learned_outlet_positions.json"
        self.config_file = "gui_config.json"
        self.learned_data = {}
        self.learned_outlet_positions = {}
        try:
            with open(self.learned_positions_file, 'r') as f:
                self.learned_data = json.load(f)
            self.log_message(f"Successfully loaded {len(self.learned_data)} learned configurations.")
        except (FileNotFoundError, json.JSONDecodeError):
            self.log_message("No learned positions file found. Starting fresh.")
            self.learned_data = {}

        # --- Data Histories for Plotting ---
        self.live_pressure_var = tk.StringVar(value="---.-- Torr")
        self.live_time_history = collections.deque(maxlen=500)
        self.live_std_pressure_history = collections.deque(maxlen=500)
        self.live_dut_pressure_history = {i: collections.deque(maxlen=500) for i in range(4)}

        # --- Initialize Plot Attributes ---
        self.live_std_plot = None
        self.live_dut_plots = {}
        self.ax_inlet_valve = None
        self.ax_outlet_valve = None


        self.manual_trace_time = collections.deque(maxlen=200)
        self.manual_trace_std = collections.deque(maxlen=200)
        self.manual_trace_duts = {i: collections.deque(maxlen=200) for i in range(4)}
        self.manual_focus_device = tk.StringVar(value="std")
        self.manual_focus_channel = None

        self.manual_dut_diff_var = tk.StringVar(value="--.---- Torr")
        self.manual_dut_diff_history = collections.deque(maxlen=10)

        self.debug_full_time = []
        self.debug_full_std_pressure = []
        self.debug_full_dut_pressure = {i: [] for i in range(4)}
        self.debug_full_inlet_pos = []
        self.debug_full_outlet_pos = []

        self.setup_ui()
        self._load_gui_config()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after_id = self.after(100, self.periodic_update)

    def setup_ui(self):
        """Creates and arranges all the widgets in the main window."""
        top_config_frame = tk.Frame(self)
        top_config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        for i in range(6): top_config_frame.columnconfigure(i, weight=1)

        action_frame = tk.LabelFrame(top_config_frame, text="System Control", padx=10, pady=10)
        action_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        self.connect_button = tk.Button(action_frame, text="Connect", command=self.connect_instruments, width=15)
        self.connect_button.pack(pady=2)
        self.manual_cal_button = tk.Button(action_frame, text="Manual Cal", command=self.toggle_manual_mode, state=tk.DISABLED, width=15)
        self.manual_cal_button.pack(pady=2)
        self.start_button = tk.Button(action_frame, text="Start Auto Cal", command=self.start_calibration_thread, state=tk.DISABLED, width=15)
        self.start_button.pack(pady=2)
        self.e_stop_button = tk.Button(action_frame, text="E-Stop", command=self.e_stop_action, bg="red", fg="white", state=tk.DISABLED, width=15)
        self.e_stop_button.pack(pady=2)
        self.resume_button = tk.Button(action_frame, text="Resume", command=self.resume_from_estop, bg="orange", state=tk.DISABLED, width=15)
        self.resume_button.pack(pady=2)
        
        custom_sp_frame = tk.Frame(action_frame)
        custom_sp_frame.pack(pady=5)
        tk.Label(custom_sp_frame, text="Setpoint:").pack(side=tk.LEFT)
        self.custom_pressure_entry_main = tk.Entry(custom_sp_frame, width=8)
        self.custom_pressure_entry_main.pack(side=tk.LEFT, padx=2)
        self.set_pressure_button = tk.Button(custom_sp_frame, text="Set", command=self._set_custom_pressure_main, state=tk.DISABLED, width=4)
        self.set_pressure_button.pack(side=tk.LEFT)
        
        config_frame = tk.LabelFrame(top_config_frame, text="Configuration", padx=10, pady=10)
        config_frame.grid(row=0, column=1, sticky="nsew", padx=(0, 5))

        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        valid_ranges = sorted([0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 500.0, 1000.0])

        tk.Label(config_frame, text="Inlet (Inv):").grid(row=0, column=0, sticky="e")
        self.inlet_com_var = tk.StringVar(self)
        self.inlet_com_combo = ttk.Combobox(config_frame, textvariable=self.inlet_com_var, values=com_ports, width=8)
        self.inlet_com_combo.grid(row=0, column=1, sticky="w")

        tk.Label(config_frame, text="Outlet (Dir):").grid(row=1, column=0, sticky="e")
        self.outlet_com_var = tk.StringVar(self)
        self.outlet_com_combo = ttk.Combobox(config_frame, textvariable=self.outlet_com_var, values=com_ports, width=8)
        self.outlet_com_combo.grid(row=1, column=1, sticky="w")
        
        tk.Label(config_frame, text="Turbo:").grid(row=2, column=0, sticky="e")
        self.turbo_com_var = tk.StringVar(self)
        self.turbo_com_combo = ttk.Combobox(config_frame, textvariable=self.turbo_com_var, values=com_ports, width=8)
        self.turbo_com_combo.grid(row=2, column=1, sticky="w")
        
        tk.Label(config_frame, text="DAQ:").grid(row=3, column=0, sticky="e")
        self.daq_com_var = tk.StringVar(self)
        self.daq_com_combo = ttk.Combobox(config_frame, textvariable=self.daq_com_var, values=com_ports, width=8)
        self.daq_com_combo.grid(row=3, column=1, sticky="w")

        tk.Label(config_frame, text="System FS:").grid(row=4, column=0, sticky="e")
        self.std_fs_var = tk.StringVar(self)
        self.std_fs_menu = tk.OptionMenu(config_frame, self.std_fs_var, *valid_ranges)
        self.std_fs_menu.grid(row=4, column=1, sticky="w")

        dut_frame = tk.LabelFrame(top_config_frame, text="Devices Under Test (DUTs)", padx=10, pady=10)
        dut_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 5))
        self.dut_widgets = []
        for i in range(4):
            tk.Label(dut_frame, text=f"Dev {i+1}:").grid(row=i, column=0, sticky="w")
            enabled_var = tk.BooleanVar(self)
            fs_var = tk.StringVar(self)
            check = tk.Checkbutton(dut_frame, text="En", variable=enabled_var)
            check.grid(row=i, column=1)
            label = tk.Label(dut_frame, text="FS:")
            label.grid(row=i, column=2, padx=(5,0))
            menu = tk.OptionMenu(dut_frame, fs_var, *valid_ranges)
            menu.grid(row=i, column=3)
            self.dut_widgets.append({'enabled': enabled_var, 'fs': fs_var, 'check': check, 'menu': menu})

        turbo_status_frame = tk.LabelFrame(top_config_frame, text="Turbo Pump", padx=10, pady=10)
        turbo_status_frame.grid(row=0, column=3, sticky="nsew", padx=(5, 5))
        
        readout_frame = tk.Frame(turbo_status_frame)
        readout_frame.pack()
        tk.Label(readout_frame, textvariable=self.turbo_rpm_var, font=("Helvetica", 14, "bold")).pack()
        tk.Label(readout_frame, textvariable=self.turbo_temp_var, font=("Helvetica", 10)).pack()

        self.turbo_fig, self.ax_turbo = plt.subplots(figsize=(1.0, 1.0), dpi=80)
        self.turbo_canvas = FigureCanvasTkAgg(self.turbo_fig, master=turbo_status_frame)
        self.turbo_canvas.get_tk_widget().pack(pady=2)
        self._draw_turbine(0)

        led_frame = tk.Frame(turbo_status_frame)
        led_frame.pack(pady=2)
        tk.Label(led_frame, text="Starting").grid(row=0, column=0, padx=2)
        self.led_starting = tk.Canvas(led_frame, width=15, height=15, bg='gray', highlightthickness=1)
        self.led_starting.grid(row=1, column=0, padx=2)
        tk.Label(led_frame, text="At Speed").grid(row=0, column=1, padx=2)
        self.led_at_speed = tk.Canvas(led_frame, width=15, height=15, bg='gray', highlightthickness=1)
        self.led_at_speed.grid(row=1, column=1, padx=2)
        tk.Label(led_frame, text="Fault").grid(row=0, column=2, padx=2)
        self.led_fault = tk.Canvas(led_frame, width=15, height=15, bg='gray', highlightthickness=1)
        self.led_fault.grid(row=1, column=2, padx=2)
        tk.Label(led_frame, text="Standby").grid(row=0, column=3, padx=2)
        self.led_standby = tk.Canvas(led_frame, width=15, height=15, bg='gray', highlightthickness=1)
        self.led_standby.grid(row=1, column=3, padx=2)
        
        pump_control_frame = tk.Frame(turbo_status_frame)
        pump_control_frame.pack(pady=2)
        self.start_pump_button = tk.Button(pump_control_frame, text="Start", command=self._start_pump, state=tk.DISABLED, width=6)
        self.start_pump_button.pack(side=tk.LEFT)
        self.stop_pump_button = tk.Button(pump_control_frame, text="Stop", command=self._stop_pump, state=tk.DISABLED, width=6)
        self.stop_pump_button.pack(side=tk.LEFT)
        self.standby_button = tk.Button(pump_control_frame, text="Standby", command=self._enter_standby_state, state=tk.DISABLED, width=7)
        self.standby_button.pack(side=tk.LEFT, padx=(5,0))
        self.nominal_button = tk.Button(pump_control_frame, text="Nominal", command=self._enter_nominal_speed_mode, state=tk.DISABLED, width=7)
        self.nominal_button.pack(side=tk.LEFT)

        valve_status_frame = tk.LabelFrame(top_config_frame, text="Valve Control", padx=10, pady=10)
        valve_status_frame.grid(row=0, column=4, sticky="nsew", padx=(5, 5))
        valve_status_frame.columnconfigure(0, weight=1); valve_status_frame.columnconfigure(1, weight=1)

        # Outlet Controls
        tk.Label(valve_status_frame, text="Outlet Valve", font=("Helvetica", 10, "bold")).grid(row=1, column=0)
        self.outlet_pos_var = tk.StringVar(value="-- %")
        tk.Label(valve_status_frame, textvariable=self.outlet_pos_var, font=("Helvetica", 14, "bold")).grid(row=2, column=0)
        self.outlet_valve_fig, self.ax_outlet_valve = plt.subplots(figsize=(0.8, 0.8), dpi=80)
        self.outlet_valve_canvas = FigureCanvasTkAgg(self.outlet_valve_fig, master=valve_status_frame)
        self.outlet_valve_canvas.get_tk_widget().grid(row=3, column=0, pady=2)
        self._draw_valve(self.ax_outlet_valve, 0)
        
        outlet_open_frame = tk.Frame(valve_status_frame)
        outlet_open_frame.grid(row=0, column=0)
        self.fine_open_button = tk.Button(outlet_open_frame, text="Fine+", command=self._fine_bump_outlet_open, state=tk.DISABLED)
        self.fine_open_button.pack(side=tk.LEFT)
        self.coarse_open_button = tk.Button(outlet_open_frame, text="Coarse+", command=self._coarse_bump_outlet_open, state=tk.DISABLED)
        self.coarse_open_button.pack(side=tk.LEFT)

        outlet_close_frame = tk.Frame(valve_status_frame)
        outlet_close_frame.grid(row=4, column=0)
        self.fine_closed_button = tk.Button(outlet_close_frame, text="Fine-", command=self._fine_bump_outlet_closed, state=tk.DISABLED)
        self.fine_closed_button.pack(side=tk.LEFT)
        self.coarse_closed_button = tk.Button(outlet_close_frame, text="Coarse-", command=self._coarse_bump_outlet_closed, state=tk.DISABLED)
        self.coarse_closed_button.pack(side=tk.LEFT)
        self.close_valve_button = tk.Button(valve_status_frame, text="Close", command=self._close_outlet_valve, state=tk.DISABLED)
        self.close_valve_button.grid(row=5, column=0, pady=2)

        # Inlet Controls
        tk.Label(valve_status_frame, text="Inlet Valve", font=("Helvetica", 10, "bold")).grid(row=1, column=1)
        self.inlet_pos_var = tk.StringVar(value="-- %")
        tk.Label(valve_status_frame, textvariable=self.inlet_pos_var, font=("Helvetica", 14, "bold")).grid(row=2, column=1)
        self.inlet_valve_fig, self.ax_inlet_valve = plt.subplots(figsize=(0.8, 0.8), dpi=80)
        self.inlet_valve_canvas = FigureCanvasTkAgg(self.inlet_valve_fig, master=valve_status_frame)
        self.inlet_valve_canvas.get_tk_widget().grid(row=3, column=1, pady=2)
        self._draw_valve(self.ax_inlet_valve, 0)

        inlet_open_frame = tk.Frame(valve_status_frame)
        inlet_open_frame.grid(row=0, column=1)
        self.fine_inlet_open_button = tk.Button(inlet_open_frame, text="Fine+", command=self._fine_bump_inlet_open, state=tk.DISABLED)
        self.fine_inlet_open_button.pack(side=tk.LEFT)
        self.coarse_inlet_open_button = tk.Button(inlet_open_frame, text="Coarse+", command=self._coarse_bump_inlet_open, state=tk.DISABLED)
        self.coarse_inlet_open_button.pack(side=tk.LEFT)

        inlet_close_frame = tk.Frame(valve_status_frame)
        inlet_close_frame.grid(row=4, column=1)
        self.fine_inlet_closed_button = tk.Button(inlet_close_frame, text="Fine-", command=self._fine_bump_inlet_closed, state=tk.DISABLED)
        self.fine_inlet_closed_button.pack(side=tk.LEFT)
        self.coarse_inlet_closed_button = tk.Button(inlet_close_frame, text="Coarse-", command=self._coarse_bump_inlet_closed, state=tk.DISABLED)
        self.coarse_inlet_closed_button.pack(side=tk.LEFT)
        self.close_inlet_valve_button = tk.Button(valve_status_frame, text="Close", command=self._close_inlet_valve, state=tk.DISABLED)
        self.close_inlet_valve_button.grid(row=5, column=1, pady=2)


        pressure_readout_frame = tk.LabelFrame(top_config_frame, text="Live System Pressure", padx=10, pady=10)
        pressure_readout_frame.grid(row=0, column=5, sticky="nsew")
        pressure_readout_frame.rowconfigure(0, weight=1)
        pressure_readout_frame.columnconfigure(0, weight=1)
        tk.Label(pressure_readout_frame, textvariable=self.live_pressure_var, font=("Helvetica", 22, "bold")).grid(row=0, column=0, sticky="nsew")

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
        self.command_entry.pack(fill=tk.X, expand=True, side=tk.LEFT, padx=(0,10))
        self.command_entry.bind("<Return>", self.send_manual_inlet_command)

        tk.Label(input_frame, text="Outlet CMD:").pack(side=tk.LEFT)
        self.outlet_command_entry = tk.Entry(input_frame, font=("Courier", 10), bg="#2c2c2c", fg="#00ff00", insertbackground="#00ff00")
        self.outlet_command_entry.pack(fill=tk.X, expand=True, side=tk.LEFT, padx=(0,5))
        self.outlet_command_entry.bind("<Return>", self.send_manual_outlet_command)

        self.cmd_ref_button = tk.Button(input_frame, text="CMD Ref", command=self.show_command_reference)
        self.cmd_ref_button.pack(side=tk.LEFT, padx=5)

    def _shutdown_system(self):
        self.log_message("EMERGENCY SHUTDOWN INITIATED...")
        if self.state_controller: self.state_controller.close_valves()
        if self.turbo_controller: 
            self.turbo_controller.send_command("TMPOFF")
            
        self.start_button.config(state=tk.DISABLED)
        
    def _start_pump(self):
        if self.turbo_controller:
            self.log_message("Sending start command to turbo pump...")
            self.turbo_controller.send_command("TMPON")

    def _stop_pump(self):
        if self.turbo_controller:
            self.log_message("Sending stop command to turbo pump...")
            self.turbo_controller.send_command("TMPOFF")

    def _enter_standby_state(self):
        self.log_message("Entering standby mode...")
        if self.state_controller: self.state_controller.close_valves()
        if self.turbo_controller: self.turbo_controller.send_command("SBY")
        
    def _enter_nominal_speed_mode(self):
        self.log_message("Commanding pump to nominal speed...")
        if self.turbo_controller:
            self.turbo_controller.send_command("TMPON")
            self.turbo_controller.send_command("NSP")


    def _draw_turbine(self, rpm):
        self.ax_turbo.clear()
        self.ax_turbo.set_xlim(-1.5, 1.5); self.ax_turbo.set_ylim(-1.5, 1.5)
        self.ax_turbo.axis('off')

        if rpm < 100:
            rotation_speed = 0
            blade_color = self.stopped_color
        elif rpm <= 11500:
            rotation_speed = (rpm / 11500.0) * 2.0
            blade_color = self.stopped_color
        elif 11500 < rpm < 25000:
            factor = (rpm - 11500) / (25000 - 11500)
            rotation_speed = 2.0 + factor * 10.0
            blade_color = self.starting_color
        else:
            factor = np.clip((rpm - 25000) / (27000 - 25000), 0.0, 1.0)
            rotation_speed = 12.0 + factor * 13.0
            
            yellow_rgb = np.array([int(self.starting_color[i:i+2], 16) for i in (1, 3, 5)]) / 255.0
            green_rgb = np.array([int(self.at_speed_color[i:i+2], 16) for i in (1, 3, 5)]) / 255.0
            blade_color = yellow_rgb + (green_rgb - yellow_rgb) * factor
        
        angle = (time.time() * rotation_speed * 10) % 360
        
        for i in range(8):
            blade = patches.Wedge((0,0), 1.2, (i*45)-15 + angle, (i*45)+15 + angle, facecolor=blade_color, edgecolor='black', linewidth=0.5)
            self.ax_turbo.add_patch(blade)
        
        hub = patches.Circle((0, 0), 0.4, facecolor='#5a5a5a', edgecolor='black')
        self.ax_turbo.add_patch(hub)
        
        self.turbo_canvas.draw_idle()

    def _fine_bump_outlet_open(self):
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = min(self.state_controller.outlet_valve_pos + 0.1, 100.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped open to {new_pos:.2f}%")

    def _fine_bump_outlet_closed(self):
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = max(self.state_controller.outlet_valve_pos - 0.1, 0.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped closed to {new_pos:.2f}%")
            
    def _coarse_bump_outlet_open(self):
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = min(self.state_controller.outlet_valve_pos + 1.0, 100.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped open to {new_pos:.2f}%")

    def _coarse_bump_outlet_closed(self):
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = max(self.state_controller.outlet_valve_pos - 1.0, 0.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped closed to {new_pos:.2f}%")
            
    def _close_outlet_valve(self):
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller._write_to_outlet("C")
            self.log_message("BUMP -> Outlet valve closed.")

    def _fine_bump_inlet_open(self):
        self._bump_inlet_valve(0.1)

    def _coarse_bump_inlet_open(self):
        self._bump_inlet_valve(1.0)

    def _fine_bump_inlet_closed(self):
        self._bump_inlet_valve(-0.1)

    def _coarse_bump_inlet_closed(self):
        self._bump_inlet_valve(-1.0)
        
    def _close_inlet_valve(self):
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller._write_to_inlet("C")
            self.log_message("BUMP -> Inlet valve commanded to close.")

    def _bump_inlet_valve(self, amount):
        if self.state_controller and self.state_controller.is_connected and self.state_controller.inlet_valve_pos is not None:
            # Remember inlet is inverse, so adding to open means subtracting from position
            current_pos = self.state_controller.inlet_valve_pos
            new_pos = max(0.0, min(100.0, current_pos - amount))
            self.state_controller._write_to_inlet(f"S5 {new_pos:.2f}") # Setpoint E
            self.state_controller._write_to_inlet("D5") # Activate Setpoint E
            self.log_message(f"BUMP -> Inlet bumped to {100-new_pos:.2f}% open.")


    def _set_custom_pressure_main(self):
        if not self.state_controller or not self.state_controller.is_connected:
            self.log_message("Controller not connected. Cannot set pressure.")
            return
        try:
            pressure = float(self.custom_pressure_entry_main.get())
            if not (0 <= pressure <= self.standard_fs_value):
                messagebox.showerror("Invalid Input", f"Pressure must be between 0 and {self.standard_fs_value} Torr.")
                return

            predicted_pos = self._predict_outlet_position(pressure)
            threading.Thread(target=self.state_controller.set_pressure, args=(pressure, predicted_pos), daemon=True).start()
            self.log_message(f"Custom setpoint of {pressure:.3f} Torr sent to controller.")

        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for the pressure.")

    def show_command_reference(self):
        win = tk.Toplevel(self)
        win.title("Command and Request Reference")
        win.geometry("800x700")

        text_area = scrolledtext.ScrolledText(win, wrap=tk.WORD, font=("Courier", 10))
        text_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        text_area.tag_configure("header", font=("Courier", 10, "bold"))

        try:
            with open("command_reference.txt", 'r') as f:
                for line in f:
                    if line.strip().startswith("##"):
                        text_area.insert(tk.END, line.lstrip("# "), "header")
                    else:
                        text_area.insert(tk.END, line)
        except FileNotFoundError:
            text_area.insert(tk.INSERT, "Error: command_reference.txt not found.")

        text_area.config(state=tk.DISABLED)

        close_button = tk.Button(win, text="Close", command=win.destroy)
        close_button.pack(pady=5)
        win.transient(self)
        win.grab_set()

    def _draw_valve(self, ax, position_percent):
        ax.clear()
        ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2)
        ax.axis('off')

        valve_body = patches.Circle((0, 0), 1, facecolor='#c0c0c0', edgecolor='black', linewidth=1.5)
        ax.add_patch(valve_body)

        normalized_pos = position_percent / 100.0
        scaled_pos = normalized_pos ** 0.5
        final_angle_deg = scaled_pos * 90.0

        ellipse_width = 2 * np.cos(np.deg2rad(final_angle_deg))
        butterfly = patches.Ellipse((0,0), width=ellipse_width, height=2, facecolor='#5a5a5a', edgecolor='black')
        transform = transforms.Affine2D().rotate_deg(90 - final_angle_deg) + ax.transData
        butterfly.set_transform(transform)
        ax.add_patch(butterfly)

        if ax == self.ax_inlet_valve:
            self.inlet_valve_canvas.draw_idle()
        elif ax == self.ax_outlet_valve:
            self.outlet_valve_canvas.draw_idle()

    def send_manual_inlet_command(self, event=None):
        command = self.command_entry.get().strip()
        self.command_entry.delete(0, tk.END)
        if not command: return
        self.log_message(f"> INLET: {command}")
        if self.state_controller and self.state_controller.is_connected:
            response = self.state_controller._query_inlet(command)
            if response: self.log_message(f"  Response: {response}")
            else: self.log_message("  Command sent (no response).")
        else:
            self.log_message("Controller not connected.")

    def send_manual_outlet_command(self, event=None):
        command = self.outlet_command_entry.get().strip()
        self.outlet_command_entry.delete(0, tk.END)
        if not command: return
        self.log_message(f"> OUTLET: {command}")
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller._write_to_outlet(command)
            self.log_message("  Command sent.")
        else:
            self.log_message("Controller not connected.")

    def connect_instruments(self):
        try:
            self.e_stop_triggered_event.clear()
            self.standard_fs_value = float(self.std_fs_var.get())
            self.state_controller = StateMachinePressureController(
                inlet_port=self.inlet_com_var.get(),
                outlet_port=self.outlet_com_var.get(),
                full_scale_pressure=self.standard_fs_value,
                log_queue=self.log_queue,
                e_stop_event=self.e_stop_triggered_event
            )
            self.log_message(f"Connected to Controllers on {self.inlet_com_var.get()} & {self.outlet_com_var.get()}.")
            self.state_controller.start()

            self.daq = DAQController(self.daq_com_var.get())
            self.log_message(f"Connected to DAQ on {self.daq_com_var.get()}.")
            
            self.turbo_controller = TurboController(self.turbo_com_var.get(), self.log_queue)
            self.turbo_controller.start()
            
            time.sleep(1.2)
            
            flags = self.turbo_controller.status_flags
            if not flags['is_on']:
                self.log_message("Turbo pump is off. Sending start command...")
                self.turbo_controller.send_command("TMPON")
                time.sleep(1)
            
            flags = self.turbo_controller.status_flags
            if flags['standby']:
                self.log_message("Turbo pump in stand-by mode. Setting turbo to nominal speed...")
                self.turbo_controller.send_command("NSP")
            else:
                self.log_message("✅ Turbo pump is already at nominal speed.")


            self.active_duts = [{'channel': i, 'fs': float(w['fs'].get())} for i, w in enumerate(self.dut_widgets) if w['enabled'].get()]
            if not self.active_duts: raise ValueError("At least one DUT must be enabled.")

            fs_key = str(self.standard_fs_value)
            raw_data = self.learned_data.get(fs_key, {})
            self.learned_outlet_positions = {float(k): v for k, v in raw_data.items()}
            self.log_message(f"Activated learning profile for {fs_key} Torr FS ({len(self.learned_outlet_positions)} points).")

            self.start_time = time.time()
            self.start_button.config(state=tk.NORMAL)
            self.manual_cal_button.config(state=tk.NORMAL)
            self.standby_button.config(state=tk.NORMAL)
            self.nominal_button.config(state=tk.NORMAL)
            self.start_pump_button.config(state=tk.NORMAL)
            self.stop_pump_button.config(state=tk.NORMAL)
            self.connect_button.config(state=tk.DISABLED)
            self.set_config_state(tk.DISABLED)
            self.set_pressure_button.config(state=tk.NORMAL)
            
            for btn in [self.fine_open_button, self.fine_closed_button, self.coarse_open_button, self.coarse_closed_button, self.close_valve_button, self.fine_inlet_open_button, self.coarse_inlet_open_button, self.fine_inlet_closed_button, self.coarse_inlet_closed_button, self.close_inlet_valve_button]:
                btn.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.NORMAL)
            self.configure_plots()
        except (ValueError, ConnectionError) as e:
            self.log_message(f"ERROR: {e}")

    def set_config_state(self, state):
        self.inlet_com_combo.config(state=state); self.outlet_com_combo.config(state=state)
        self.std_fs_menu.config(state=state); self.daq_com_combo.config(state=state)
        self.turbo_com_combo.config(state=state)
        for widget_set in self.dut_widgets:
            widget_set['check'].config(state=state); widget_set['menu'].config(state=state)

    def e_stop_action(self):
        self.e_stop_triggered_event.set()
        if self.is_calibrating or self.is_in_manual_mode:
            self.is_calibrating = False
            self.log_message("\n*** E-STOP ***\nProcess stopped. Saving learned data...")
            self._save_learned_data()

            if self.is_in_manual_mode:
                self.toggle_manual_mode()
            
            self._shutdown_system()
        else:
            self._shutdown_system()

        # Disable controls and enable resume
        self.start_button.config(state=tk.DISABLED)
        self.manual_cal_button.config(state=tk.DISABLED)
        self.resume_button.config(state=tk.NORMAL)


    def resume_from_estop(self):
        self.log_message("Resuming operations...")
        self.e_stop_triggered_event.clear()
        
        # Re-enable controls
        self.start_button.config(state=tk.NORMAL)
        self.manual_cal_button.config(state=tk.NORMAL)
        self.resume_button.config(state=tk.DISABLED)
        
        # Restart the controller polling loops
        if self.state_controller:
            self.state_controller.start()


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
        try:
            current_time = time.time() - self.start_time if self.start_time > 0 else -1

            while not self.log_queue.empty():
                msg = self.log_queue.get()
                timestamp = f"[{current_time: >7.2f}s]" if current_time >= 0 else "[  --.--s]"
                self.terminal_text.config(state=tk.NORMAL)
                self.terminal_text.insert(tk.END, f"\n{timestamp} {msg}")
                self.terminal_text.see(tk.END)
                self.terminal_text.config(state=tk.DISABLED)

            if self.turbo_controller and self.turbo_controller.is_connected:
                self.turbo_rpm_var.set(f"{self.turbo_controller.rpm} RPM")
                self.turbo_temp_var.set(f"{self.turbo_controller.pump_temp}°C")
                self._draw_turbine(self.turbo_controller.rpm)

                flags = self.turbo_controller.status_flags
                self.led_starting.config(bg=self.starting_color if flags['is_on'] and not flags['at_speed'] and not flags['standby'] else 'gray')
                self.led_at_speed.config(bg=self.at_speed_color if flags['at_speed'] else 'gray')
                self.led_fault.config(bg='red' if flags['fault'] else 'gray')
                self.led_standby.config(bg=self.starting_color if flags['standby'] else 'gray')


            if self.state_controller and self.state_controller.is_connected:
                std_pressure = self.state_controller.current_pressure
                inlet_pos = self.state_controller.inlet_valve_pos
                outlet_pos = self.state_controller.outlet_valve_pos

                self.live_pressure_var.set(f"{std_pressure:.3f} Torr" if std_pressure is not None else "---.-- Torr")

                self.live_time_history.append(current_time)
                self.live_std_pressure_history.append(std_pressure)
                if self.is_in_manual_mode:
                    self.manual_trace_time.append(current_time)
                    self.manual_trace_std.append(std_pressure)

                if self.is_calibrating:
                    self.debug_full_time.append(current_time)
                    self.debug_full_std_pressure.append(std_pressure)
                    self.debug_full_inlet_pos.append(inlet_pos)
                    self.debug_full_outlet_pos.append(outlet_pos)

                display_inlet_pos = 100.0 - inlet_pos if inlet_pos is not None else 0.0
                display_outlet_pos = outlet_pos if outlet_pos is not None else 0.0
                self.inlet_pos_var.set(f"{display_inlet_pos:.1f} %")
                self.outlet_pos_var.set(f"{display_outlet_pos:.1f} %")
                self._draw_valve(self.ax_inlet_valve, display_inlet_pos)
                self._draw_valve(self.ax_outlet_valve, display_outlet_pos)

                for i in range(4):
                    dut_pressure = np.nan
                    if any(d['channel'] == i for d in self.active_duts):
                        voltage = self.daq.read_voltage(i) if self.daq else None
                        if voltage is not None:
                            fs = [d['fs'] for d in self.active_duts if d['channel'] == i][0]
                            dut_pressure = voltage * (fs / 9.9)

                    self.live_dut_pressure_history[i].append(dut_pressure)
                    if self.is_in_manual_mode:
                        self.manual_trace_duts[i].append(dut_pressure)
                    if self.is_calibrating:
                        self.debug_full_dut_pressure[i].append(dut_pressure)

                if not self.is_in_manual_mode and self.live_std_plot:
                    self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
                    for i, line in self.live_dut_plots.items():
                        is_active = any(d['channel'] == i for d in self.active_duts)
                        is_completed = i in self.completed_duts
                        line.set_visible(is_active and not is_completed)
                        line.set_data(self.live_time_history, self.live_dut_pressure_history[i])

                    t_max_main = self.live_time_history[-1] if self.live_time_history else 0
                    self.ax_live_pressure.set_xlim(max(0, t_max_main - 90), t_max_main + 1)
                    self.ax_live_pressure.relim(); self.ax_live_pressure.autoscale_view(scaley=True)
                    self.canvas.draw_idle()

                if self.is_in_manual_mode and self.manual_frame:
                    t_max_manual = self.manual_trace_time[-1] if self.manual_trace_time else 0
                    if self.manual_focus_channel is None:
                        for i, plot_data in enumerate(self.manual_frame.manual_quadrant_lines):
                            ax = plot_data['ax']
                            if ax.get_visible():
                                plot_data['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                                plot_data['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[i])
                                ax.relim(); ax.autoscale_view(scaley=True)
                        self.manual_frame.axs_manual_quad.flat[0].set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                        self.manual_frame.manual_quad_canvas.draw_idle()
                    else:
                        ch = self.manual_focus_channel
                        self.manual_frame.manual_single_lines['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                        self.manual_frame.manual_single_lines['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[ch])
                        self.manual_frame.ax_manual_single.set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                        self.manual_frame.ax_manual_single.relim(); self.manual_frame.ax_manual_single.autoscale_view(scaley=True)
                        self.manual_frame.manual_single_canvas.draw_idle()

                    diff = np.nan
                    ch = self.manual_focus_channel
                    if ch is not None:
                        std_p = self.live_std_pressure_history[-1] if self.live_std_pressure_history and self.live_std_pressure_history[-1] is not None else np.nan
                        dut_p = self.live_dut_pressure_history[ch][-1] if self.live_dut_pressure_history[ch] and not np.isnan(self.live_dut_pressure_history[ch][-1]) else np.nan

                        if not np.isnan(std_p) and not np.isnan(dut_p):
                            diff = dut_p - std_p
                            self.manual_dut_diff_var.set(f"{diff:+.4f} Torr")
                            self.manual_dut_diff_history.append(diff)
                        else:
                            self.manual_dut_diff_var.set("--.---- Torr")
                    else:
                        self.manual_dut_diff_var.set("N/A")
                        self.manual_dut_diff_history.clear()

                    self.manual_frame.update_diff_indicator(diff, list(self.manual_dut_diff_history))

        except Exception as e:
            self.log_message(f"ERROR in periodic_update: {e}")
            import traceback
            traceback.print_exc()

        self.after_id = self.after(200, self.periodic_update)

    def toggle_manual_mode(self):
        if not self.is_in_manual_mode:
            threading.Thread(target=self._enter_manual_mode_with_checks, daemon=True).start()
        else:
            self.is_in_manual_mode = False
            self.manual_cal_button.config(text="Manual Cal")
            self.start_button.config(state=tk.NORMAL)
            self.standby_button.config(state=tk.NORMAL)
            self.nominal_button.config(state=tk.NORMAL)
            self.set_pressure_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.NORMAL)

            if self.manual_frame:
                self.manual_frame.stop_learning_thread()
                self.manual_frame.destroy()
                self.manual_frame = None
            self.canvas.get_tk_widget().grid()
            self._save_learned_data()
            self._enter_standby_state()

    def _wait_for_turbo_ready(self, operation_name="Operation"):
        if self.turbo_controller:
            flags = self.turbo_controller.status_flags
            is_ready = flags['at_speed'] and not flags['standby']
            
            if not is_ready:
                self.log_message(f"Waiting for turbo pump to be ready for {operation_name}...")
                
                if not flags['is_on']:
                    self.log_message("Pump is off. Sending start command...")
                    self.turbo_controller.send_command("TMPON")
                    time.sleep(1)
                
                self.turbo_controller.send_command("NSP")
                
                while not self.e_stop_triggered_event.is_set():
                    flags = self.turbo_controller.status_flags
                    if flags['at_speed'] and not flags['standby']:
                        self.log_message(f"✅ Turbo pump is ready. Starting {operation_name}.")
                        return True
                    
                    current_operation_active = self.is_calibrating or self.is_in_manual_mode
                    if not current_operation_active:
                        self.log_message(f"{operation_name} canceled by user.")
                        return False
                    time.sleep(2)
        return True

    def _enter_manual_mode_with_checks(self):
        self.is_in_manual_mode = True 
        self.after(0, self._setup_manual_ui)
        threading.Thread(target=self._wait_for_turbo_and_set_initial_pressure, daemon=True).start()

    def _wait_for_turbo_and_set_initial_pressure(self):
        if self._wait_for_turbo_ready("Manual Mode"):
            if self.manual_frame and not self.e_stop_triggered_event.is_set():
                self.after(0, self.manual_frame.set_manual_pressure, 0)

    def _setup_manual_ui(self):
        self.manual_cal_button.config(text="Exit Manual Cal")
        self.start_button.config(state=tk.DISABLED)
        self.standby_button.config(state=tk.DISABLED)
        self.nominal_button.config(state=tk.DISABLED)
        self.set_pressure_button.config(state=tk.DISABLED)
        self.e_stop_button.config(state=tk.NORMAL)

        self.canvas.get_tk_widget().grid_remove()
        self.manual_frame = ManualCalFrame(self.plot_term_frame, self)
        self.manual_frame.grid(row=0, column=0, sticky="nsew")

    def start_calibration_thread(self):
        self.is_calibrating = True
        self.debug_full_time.clear(); self.debug_full_std_pressure.clear()
        self.debug_full_inlet_pos.clear(); self.debug_full_outlet_pos.clear()
        for i in range(4): self.debug_full_dut_pressure[i].clear()

        self.log_message("\n--- Starting Automated Data Logging ---")
        self.data_storage = {'Setpoint_Torr': [], 'Standard_Pressure_Torr': []}
        for dut in self.active_duts:
            self.data_storage[f'Device_{dut["channel"]+1}_Pressure_Torr'] = []
        self.error_plot_data.clear()

        self.start_button.config(state=tk.DISABLED); self.manual_cal_button.config(state=tk.DISABLED)
        self.standby_button.config(state=tk.DISABLED)
        self.nominal_button.config(state=tk.DISABLED)
        self.set_pressure_button.config(state=tk.DISABLED)
        self.e_stop_button.config(state=tk.NORMAL)
        
        threading.Thread(target=self._run_auto_cal_with_checks, daemon=True).start()

    def _run_auto_cal_with_checks(self):
        if not self._wait_for_turbo_ready("Auto Calibration"):
            self.is_calibrating = False 
            return 
        run_calibration(self)


    def _predict_outlet_position(self, target_sp):
        if not self.learned_outlet_positions: return None

        avg_positions = {float(sp): sum(pos_list) / len(pos_list) for sp, pos_list in self.learned_outlet_positions.items() if pos_list}
        if not avg_positions: return None

        known_sps = sorted(avg_positions.keys())
        if target_sp in known_sps: return avg_positions[target_sp]
        if len(known_sps) < 2: return avg_positions[min(known_sps, key=lambda sp: abs(sp - target_sp))]

        if target_sp < known_sps[0]: sp1, sp2 = known_sps[0], known_sps[1]
        elif target_sp > known_sps[-1]: sp1, sp2 = known_sps[-2], known_sps[-1]
        else:
            sp_high = min(sp for sp in known_sps if sp > target_sp)
            sp_low = max(sp for sp in known_sps if sp < target_sp)
            sp1, sp2 = sp_low, sp_high

        if sp2 == sp1: return avg_positions[sp1]
        pos1, pos2 = avg_positions[sp1], avg_positions[sp2]
        fraction = (target_sp - sp1) / (sp2 - sp1)
        return pos1 + fraction * (pos2 - pos1)

    def _save_gui_config(self):
        config_data = {
            'inlet_com': self.inlet_com_var.get(),
            'outlet_com': self.outlet_com_var.get(),
            'daq_com': self.daq_com_var.get(),
            'turbo_com': self.turbo_com_var.get(),
            'std_fs': self.std_fs_var.get(),
            'duts': [
                {'enabled': w['enabled'].get(), 'fs': w['fs'].get()}
                for w in self.dut_widgets
            ]
        }
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config_data, f, indent=4)
            self.log_message("GUI configuration saved.")
        except Exception as e:
            self.log_message(f"Error saving GUI config: {e}")

    def _load_gui_config(self):
        try:
            with open(self.config_file, 'r') as f:
                config_data = json.load(f)
            self.log_message("Loaded GUI configuration from file.")
        except (FileNotFoundError, json.JSONDecodeError):
            self.log_message("No valid config file found, using defaults.")
            config_data = {
                'inlet_com': 'COM5', 'outlet_com': 'COM6', 'daq_com': 'COM7',
                'turbo_com': 'COM8', 'std_fs': '100.0',
                'duts': [{'enabled': True, 'fs': '100.0'}] * 4
            }

        self.inlet_com_var.set(config_data.get('inlet_com', ''))
        self.outlet_com_var.set(config_data.get('outlet_com', ''))
        self.daq_com_var.set(config_data.get('daq_com', ''))
        self.turbo_com_var.set(config_data.get('turbo_com', ''))
        self.std_fs_var.set(config_data.get('std_fs', '100.0'))
        
        dut_configs = config_data.get('duts', [])
        for i, widget in enumerate(self.dut_widgets):
            if i < len(dut_configs):
                widget['enabled'].set(dut_configs[i].get('enabled', True))
                widget['fs'].set(dut_configs[i].get('fs', '100.0'))
            else:
                widget['enabled'].set(True)
                widget['fs'].set('100.0')

    def _save_learned_data(self):
        if not hasattr(self, 'standard_fs_value'): return
        try:
            fs_key = str(self.standard_fs_value)
            self.learned_data[fs_key] = {str(k): v for k, v in self.learned_outlet_positions.items()}
            with open(self.learned_positions_file, 'w') as f:
                json.dump(self.learned_data, f, indent=4)
            self.log_message(f"Saved {len(self.learned_outlet_positions)} learned points for {fs_key} Torr FS.")
        except Exception as e:
            self.log_message(f"ERROR: Could not save learned positions: {e}")

    def _generate_debug_plot(self):
        self.log_message("Generating full-run debug plot...")
        try:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
            fig.suptitle('Full Calibration Run - Debug Trace', fontsize=16)

            ax1.set_title('Pressure vs. Time'); ax1.set_ylabel('Pressure (Torr)'); ax1.grid(True, linestyle=':')
            ax1.plot(self.debug_full_time, self.debug_full_std_pressure, label='Standard', color='blue', linewidth=2)
            for dut in self.active_duts:
                ch = dut['channel']
                ax1.plot(self.debug_full_time, self.debug_full_dut_pressure[ch], label=f'DUT {ch+1}', color=self.dut_colors[ch], alpha=0.8)
            ax1.legend()

            ax2.set_title('Valve Position vs. Time'); ax2.set_ylabel('Position (% Open)'); ax2.set_xlabel('Time (s)')
            ax2.grid(True, linestyle=':'); ax2.set_ylim(-5, 105)
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
        suggestion_box = scrolledtext.ScrolledText(main_frame, font=("Courier", 10), wrap=tk.WORD, relief=tk.SOLID, borderwidth=1)
        suggestion_box.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        suggestion_box.insert(tk.END, suggestions_text); suggestion_box.config(state=tk.DISABLED)
        tk.Button(main_frame, text="Close", command=win.destroy).pack()

    def analyze_and_suggest_tuning(self):
        final_suggestion_text, plot_filenames = generate_tuning_suggestions(self.data_storage, self.active_duts)
        if final_suggestion_text:
            self.log_message(final_suggestion_text)
            for fname in plot_filenames:
                self.log_message(f"Tuning curve plot saved to '{fname}'")
            self.show_tuning_suggestions_window(final_suggestion_text)

    def update_error_plot(self):
        all_setpoints = sorted(self.error_plot_data.keys())
        if not all_setpoints: return
        self._setup_error_plot(all_setpoints)

        num_duts = len(self.active_duts)
        bar_height = min(np.diff(all_setpoints)) * 0.8 if len(all_setpoints) > 1 else 1.0
        bar_height = min(bar_height / num_duts, 2.0)

        for sp, errors_data in self.error_plot_data.items():
            max_time_for_sp = 0
            for i, dut in enumerate(self.active_duts):
                ch, fs = dut['channel'], dut['fs']
                offset = (i - (num_duts - 1) / 2) * bar_height

                if ch in errors_data:
                    error_info = errors_data[ch]
                    max_time_for_sp = max(max_time_for_sp, error_info['time'])
                    self.ax_error.barh(sp + offset, error_info['error'], height=bar_height, color=self.dut_colors[ch], edgecolor='black', linewidth=0.5)

                    if abs(error_info['error']) > (fs * 0.005):
                        ha = 'left' if error_info['error'] >= 0 else 'right'
                        self.ax_error.text(error_info['error'], sp + offset, f" ⚠️ DUT {ch+1}", va='center', ha=ha, fontsize=7, color='red', weight='bold')

            if max_time_for_sp > 0:
                trans = transforms.blended_transform_factory(self.ax_error.transAxes, self.ax_error.transData)
                self.ax_error.text(1.02, sp, f"@{max_time_for_sp:.1f}s",
                                   transform=trans,
                                   va='center', ha='left', fontsize=8, color='#333333')

        self.ax_error.relim(); self.ax_error.autoscale_view(scalex=True, scaley=False)
        self.fig.tight_layout()
        self.canvas.draw_idle()


    def on_closing(self):
        self._save_gui_config()
        self._save_learned_data()
        self.is_calibrating = False; self.is_in_manual_mode = False

        if self.after_id: self.after_cancel(self.after_id)
        
        if self.turbo_controller: self.turbo_controller.send_command("SBY")
        
        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()
        if self.turbo_controller: self.turbo_controller.stop()

        self.quit()
        self.destroy()

# =================================================================================
# Main Execution Block
# =================================================================================
if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()