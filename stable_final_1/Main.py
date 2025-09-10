# -*- coding: utf-8 -*-
# ==============================================================================
# File:         Main.py
# Author:       Gemini
# Date:         September 3, 2025
# Description:  This is the main application file for the Multi-Device
#               Adaptive Calibration System. It initializes the GUI,
#               manages device connections, runs the automated calibration
#               sequence, and orchestrates the other modules.
#
# Version 91 (Command Reference Feature):
#   - Added a "CMD Ref" button to the UI.
#   - Clicking the button opens a pop-up window displaying a formatted list
#     of all RS-232 commands, read from `command_reference.txt`.
#   - Pop-up now renders markdown-style '##' headers in a bold font.
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

# =================================================================================
# Main GUI Class
# =================================================================================
class CalibrationGUI(tk.Tk):
    """
    The main application window for the calibration system.

    This class builds the user interface, handles user actions (connect, start, stop),
    displays live data, manages the high-level state of the application (e.g.,
    auto vs. manual mode), and runs the automated calibration process.
    """
    def __init__(self):
        super().__init__()

        self.title("Multi-Device Adaptive Calibration System")
        self.attributes('-fullscreen', True) # Launch in full-screen mode
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        # --- Initialize Controllers and State ---
        self.state_controller = None
        self.daq = None
        self.is_calibrating = False
        self.is_in_manual_mode = False
        self.start_time = 0
        self.after_id = None
        self.manual_frame = None # Placeholder for the manual calibration UI frame

        self.dut_colors = ['#2ca02c', '#d62728', '#9467bd', '#8c564b']

        self.data_storage = {}
        self.error_plot_data = {}
        self.log_queue = queue.Queue()

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

        # Data for the manual calibration plots (shorter history)
        self.manual_trace_time = collections.deque(maxlen=200)
        self.manual_trace_std = collections.deque(maxlen=200)
        self.manual_trace_duts = {i: collections.deque(maxlen=200) for i in range(4)}
        self.manual_focus_device = tk.StringVar(value="std")
        self.manual_focus_channel = None

        # Additions for manual tuning helper
        self.manual_dut_diff_var = tk.StringVar(value="--.---- Torr")
        self.manual_dut_diff_history = collections.deque(maxlen=10)

        # Data for the final debug plot (full history)
        self.debug_full_time = []
        self.debug_full_std_pressure = []
        self.debug_full_dut_pressure = {i: [] for i in range(4)}
        self.debug_full_inlet_pos = []
        self.debug_full_outlet_pos = []

        self.setup_ui()
        self._load_gui_config() # Load last used settings
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after_id = self.after(100, self.periodic_update)

    def setup_ui(self):
        """Creates and arranges all the widgets in the main window."""
        # --- Top Configuration Frame ---
        top_config_frame = tk.Frame(self)
        top_config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        top_config_frame.columnconfigure(0, weight=2)
        top_config_frame.columnconfigure(1, weight=2)
        top_config_frame.columnconfigure(2, weight=1)
        top_config_frame.columnconfigure(3, weight=1)

        # --- Instrument Configuration ---
        config_frame = tk.LabelFrame(top_config_frame, text="Configuration", padx=10, pady=10)
        config_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        valid_ranges = sorted([0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 500.0, 1000.0])

        tk.Label(config_frame, text="Inlet Controller (Inverse):").grid(row=0, column=0, sticky="w", columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=1, column=0, sticky="e", padx=5)
        self.inlet_com_var = tk.StringVar(self) # Default set by _load_gui_config
        self.inlet_com_combo = ttk.Combobox(config_frame, textvariable=self.inlet_com_var, values=com_ports, width=10)
        self.inlet_com_combo.grid(row=1, column=1, sticky="w")

        tk.Label(config_frame, text="Outlet Controller (Direct):").grid(row=2, column=0, sticky="w", pady=(8,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=3, column=0, sticky="e", padx=5)
        self.outlet_com_var = tk.StringVar(self)
        self.outlet_com_combo = ttk.Combobox(config_frame, textvariable=self.outlet_com_var, values=com_ports, width=10)
        self.outlet_com_combo.grid(row=3, column=1, sticky="w")

        tk.Label(config_frame, text="System FS (Torr):").grid(row=4, column=0, sticky="e", padx=5, pady=(8,0))
        self.std_fs_var = tk.StringVar(self)
        self.std_fs_menu = tk.OptionMenu(config_frame, self.std_fs_var, *valid_ranges)
        self.std_fs_menu.grid(row=4, column=1, sticky="w", pady=(8,0))

        tk.Label(config_frame, text="DAQ (RP2040):").grid(row=5, column=0, sticky="w", pady=(10,0), columnspan=2)
        tk.Label(config_frame, text="COM Port:").grid(row=6, column=0, sticky="e", padx=5)
        self.daq_com_var = tk.StringVar(self)
        self.daq_com_combo = ttk.Combobox(config_frame, textvariable=self.daq_com_var, values=com_ports, width=10)
        self.daq_com_combo.grid(row=6, column=1, sticky="w")

        # --- DUT Configuration ---
        dut_frame = tk.LabelFrame(top_config_frame, text="Devices Under Test (DUTs)", padx=10, pady=10)
        dut_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 5))
        self.dut_widgets = []
        for i in range(4):
            tk.Label(dut_frame, text=f"Device {i+1} (DAQ Ch {i}):").grid(row=i, column=0, sticky="w")
            enabled_var = tk.BooleanVar(self)
            fs_var = tk.StringVar(self)
            check = tk.Checkbutton(dut_frame, text="Enable", variable=enabled_var)
            check.grid(row=i, column=1)
            label = tk.Label(dut_frame, text="FS (Torr):")
            label.grid(row=i, column=2, padx=(10,0))
            menu = tk.OptionMenu(dut_frame, fs_var, *valid_ranges)
            menu.grid(row=i, column=3)
            self.dut_widgets.append({'enabled': enabled_var, 'fs': fs_var, 'check': check, 'menu': menu})

        # --- Live Status Indicators ---
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

        # Create figures for valve animations
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
        tk.Label(pressure_readout_frame, textvariable=self.live_pressure_var, font=("Helvetica", 22, "bold")).grid(row=0, column=0, sticky="nsew")

        # --- Main Plot and Terminal Area ---
        self.plot_term_frame = tk.Frame(self)
        self.plot_term_frame.grid(row=1, column=0, sticky="nsew")
        self.plot_term_frame.rowconfigure(0, weight=1); self.plot_term_frame.columnconfigure(0, weight=1)

        # Main figure for auto-calibration plots
        self.fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1], figure=self.fig)
        self.ax_live_pressure = self.fig.add_subplot(gs[0, 0])
        self.ax_error = self.fig.add_subplot(gs[0, 1])
        self.fig.tight_layout(pad=3.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_term_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Terminal and command entry
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

        # Add the command reference button
        self.cmd_ref_button = tk.Button(input_frame, text="CMD Ref", command=self.show_command_reference)
        self.cmd_ref_button.pack(side=tk.LEFT, padx=5)


        # --- Main Action Buttons ---
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

    def show_command_reference(self):
        """Creates a pop-up window to display the command reference with formatting."""
        win = tk.Toplevel(self)
        win.title("Command and Request Reference")
        win.geometry("800x700")

        text_area = scrolledtext.ScrolledText(win, wrap=tk.WORD, font=("Courier", 10))
        text_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        # Configure a tag for the markdown-style headers
        text_area.tag_configure("header", font=("Courier", 10, "bold"))

        try:
            with open("command_reference.txt", 'r') as f:
                for line in f:
                    # Check for "##" at the start of a stripped line
                    if line.strip().startswith("##"):
                        # Insert the line without the '## ' and apply the bold tag
                        text_area.insert(tk.END, line.lstrip("# "), "header")
                    else:
                        # Insert all other lines normally
                        text_area.insert(tk.END, line)
        except FileNotFoundError:
            text_area.insert(tk.INSERT, "Error: command_reference.txt not found.")

        text_area.config(state=tk.DISABLED) # Make it read-only

        close_button = tk.Button(win, text="Close", command=win.destroy)
        close_button.pack(pady=5)
        win.transient(self) # Keep the window on top of the main app
        win.grab_set()      # Modal behavior

    def _draw_valve(self, ax, position_percent):
        """Draws a butterfly valve animation on a given matplotlib axis."""
        ax.clear()
        ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2)
        ax.axis('off')

        valve_body = patches.Circle((0, 0), 1, facecolor='#c0c0c0', edgecolor='black', linewidth=1.5)
        ax.add_patch(valve_body)

        # Scale position non-linearly for a more realistic visual effect
        normalized_pos = position_percent / 100.0
        scaled_pos = normalized_pos ** 0.5
        final_angle_deg = scaled_pos * 90.0

        ellipse_width = 2 * np.cos(np.deg2rad(final_angle_deg))
        butterfly = patches.Ellipse((0,0), width=ellipse_width, height=2, facecolor='#5a5a5a', edgecolor='black')
        transform = transforms.Affine2D().rotate_deg(90 - final_angle_deg) + ax.transData
        butterfly.set_transform(transform)
        ax.add_patch(butterfly)

        # Redraw the specific canvas
        if ax == self.ax_inlet_valve:
            self.inlet_valve_canvas.draw_idle()
        elif ax == self.ax_outlet_valve:
            self.outlet_valve_canvas.draw_idle()

    def send_manual_inlet_command(self, event=None):
        """Sends a raw command from the entry box to the inlet controller."""
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
        """Sends a raw command from the entry box to the outlet controller."""
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
        """Establishes connections to all configured hardware."""
        try:
            self.standard_fs_value = float(self.std_fs_var.get())
            # Instantiate controller classes from imported modules
            self.state_controller = StateMachinePressureController(
                inlet_port=self.inlet_com_var.get(),
                outlet_port=self.outlet_com_var.get(),
                full_scale_pressure=self.standard_fs_value,
                log_queue=self.log_queue
            )
            self.log_message(f"Connected to Controllers on {self.inlet_com_var.get()} & {self.outlet_com_var.get()}.")
            self.state_controller.start() # Start background threads

            self.daq = DAQController(self.daq_com_var.get())
            self.log_message(f"Connected to DAQ on {self.daq_com_var.get()}.")

            # Determine which DUTs are active for this run
            self.active_duts = [{'channel': i, 'fs': float(w['fs'].get())} for i, w in enumerate(self.dut_widgets) if w['enabled'].get()]
            if not self.active_duts: raise ValueError("At least one DUT must be enabled.")

            # Load the learning profile for the selected full-scale range
            fs_key = str(self.standard_fs_value)
            raw_data = self.learned_data.get(fs_key, {})
            self.learned_outlet_positions = {float(k): v for k, v in raw_data.items()}
            self.log_message(f"Activated learning profile for {fs_key} Torr FS ({len(self.learned_outlet_positions)} points).")

            # Update UI state
            self.start_time = time.time()
            self.start_button.config(state=tk.NORMAL); self.e_stop_button.config(state=tk.DISABLED)
            self.manual_cal_button.config(state=tk.NORMAL)
            self.connect_button.config(state=tk.DISABLED)
            self.set_config_state(tk.DISABLED)
            self.configure_plots()
        except (ValueError, ConnectionError) as e:
            self.log_message(f"ERROR: {e}")

    def set_config_state(self, state):
        """Enables or disables the configuration widgets."""
        self.inlet_com_combo.config(state=state); self.outlet_com_combo.config(state=state)
        self.std_fs_menu.config(state=state); self.daq_com_combo.config(state=state)
        for widget_set in self.dut_widgets:
            widget_set['check'].config(state=state); widget_set['menu'].config(state=state)

    def e_stop_action(self):
        """Emergency stop action. Halts any running process and closes valves."""
        if self.is_calibrating or self.is_in_manual_mode:
            self.is_calibrating = False
            self.log_message("\n*** E-STOP ***\nProcess stopped. Saving learned data...")
            self._save_learned_data()

            if self.is_in_manual_mode:
                self.toggle_manual_mode() # Properly exit manual mode

            if self.state_controller: self.state_controller.close_valves()
            self.start_button.config(state=tk.NORMAL)
            self.manual_cal_button.config(state=tk.NORMAL, text="Manual Cal")
            self.e_stop_button.config(state=tk.DISABLED)

    def configure_plots(self):
        """Sets up the titles, labels, and lines for the main plots."""
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
        """Configures the error plot specifically for a calibration run."""
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
        """Puts a message into the thread-safe queue for display in the terminal."""
        self.log_queue.put(message)

    def periodic_update(self):
        """
        The main GUI update loop. Called periodically to refresh all dynamic data.
        """
        current_time = time.time() - self.start_time if self.start_time > 0 else -1

        # Process all pending log messages.
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            timestamp = f"[{current_time: >7.2f}s]" if current_time >= 0 else "[  --.--s]"
            self.terminal_text.config(state=tk.NORMAL)
            self.terminal_text.insert(tk.END, f"\n{timestamp} {msg}")
            self.terminal_text.see(tk.END)
            self.terminal_text.config(state=tk.DISABLED)

        # Update data from controllers if connected.
        if self.state_controller and self.state_controller.is_connected:
            std_pressure = self.state_controller.current_pressure
            inlet_pos = self.state_controller.inlet_valve_pos
            outlet_pos = self.state_controller.outlet_valve_pos

            self.live_pressure_var.set(f"{std_pressure:.3f} Torr" if std_pressure is not None else "---.-- Torr")

            # Update data histories
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

            # Update valve animations
            try:
                display_inlet_pos = 100.0 - inlet_pos if inlet_pos is not None else 0.0 # Inlet is inverse
                display_outlet_pos = outlet_pos if outlet_pos is not None else 0.0
                self.inlet_pos_var.set(f"{display_inlet_pos:.1f} %")
                self.outlet_pos_var.set(f"{display_outlet_pos:.1f} %")
                self._draw_valve(self.ax_inlet_valve, display_inlet_pos)
                self._draw_valve(self.ax_outlet_valve, display_outlet_pos)
            except Exception: pass

            # Read and store DUT pressures
            for i in range(4):
                dut_pressure = np.nan
                if any(d['channel'] == i for d in self.active_duts):
                    voltage = self.daq.read_voltage(i) if self.daq else None
                    if voltage is not None:
                        fs = [d['fs'] for d in self.active_duts if d['channel'] == i][0]
                        dut_pressure = voltage * (fs / 9.9) # Assuming 9.9V is DAQ FS

                self.live_dut_pressure_history[i].append(dut_pressure)
                if self.is_in_manual_mode:
                    self.manual_trace_duts[i].append(dut_pressure)
                if self.is_calibrating:
                    self.debug_full_dut_pressure[i].append(dut_pressure)

            # --- Redraw Plots ---
            # Main live plot
            self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
            for i, line in self.live_dut_plots.items():
                line.set_visible(any(d['channel'] == i for d in self.active_duts))
                line.set_data(self.live_time_history, self.live_dut_pressure_history[i])
            t_max_main = self.live_time_history[-1] if self.live_time_history else 0
            self.ax_live_pressure.set_xlim(max(0, t_max_main - 90), t_max_main + 1)
            self.ax_live_pressure.relim(); self.ax_live_pressure.autoscale_view(scaley=True)
            self.canvas.draw_idle()

            # Manual mode plots (if active)
            if self.is_in_manual_mode and self.manual_frame:
                t_max_manual = self.manual_trace_time[-1] if self.manual_trace_time else 0
                # Update quadrant plot
                if self.manual_focus_channel is None:
                    for i, plot_data in enumerate(self.manual_frame.manual_quadrant_lines):
                        ax = plot_data['ax']
                        if ax.get_visible():
                            plot_data['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                            plot_data['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[i])
                            ax.relim(); ax.autoscale_view(scaley=True)
                    self.manual_frame.axs_manual_quad.flat[0].set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                    self.manual_frame.manual_quad_canvas.draw_idle()
                # Update single focus plot
                else:
                    ch = self.manual_focus_channel
                    self.manual_frame.manual_single_lines['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                    self.manual_frame.manual_single_lines['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[ch])
                    self.manual_frame.ax_manual_single.set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                    self.manual_frame.ax_manual_single.relim(); self.manual_frame.ax_manual_single.autoscale_view(scaley=True)
                    self.manual_frame.manual_single_canvas.draw_idle()

                # --- Update Tuning Helper ---
                diff = np.nan
                ch = self.manual_focus_channel
                if ch is not None:
                    # Get latest valid pressures
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

                # Call the update function on the manual frame instance
                self.manual_frame.update_diff_indicator(diff, list(self.manual_dut_diff_history))

        self.after_id = self.after(200, self.periodic_update) # Schedule next update

    def toggle_manual_mode(self):
        """Switches the UI between auto-calibration and manual control modes."""
        self.is_in_manual_mode = not self.is_in_manual_mode
        if self.is_in_manual_mode:
            self.manual_cal_button.config(text="Exit Manual Cal")
            self.start_button.config(state=tk.DISABLED)
            self.e_stop_button.config(state=tk.NORMAL)

            # Hide the main plot canvas and show the manual control frame
            self.canvas.get_tk_widget().grid_remove()
            self.manual_frame = ManualCalFrame(self.plot_term_frame, self)
            self.manual_frame.grid(row=0, column=0, sticky="nsew")

        else:
            self.manual_cal_button.config(text="Manual Cal")
            self.start_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.DISABLED)

            # Destroy the manual frame and show the main plot canvas again
            if self.manual_frame:
                self.manual_frame.stop_learning_thread()
                self.manual_frame.destroy()
                self.manual_frame = None
            self.canvas.get_tk_widget().grid()
            if self.state_controller: self.state_controller.close_valves()

    def start_calibration_thread(self):
        """Starts the automated calibration sequence in a new thread."""
        self.is_calibrating = True
        # Clear data from previous runs
        self.debug_full_time.clear(); self.debug_full_std_pressure.clear()
        self.debug_full_inlet_pos.clear(); self.debug_full_outlet_pos.clear()
        for i in range(4): self.debug_full_dut_pressure[i].clear()

        self.log_message("\n--- Starting Automated Data Logging ---")
        self.data_storage = {'Setpoint_Torr': [], 'Standard_Pressure_Torr': []}
        for dut in self.active_duts:
            self.data_storage[f'Device_{dut["channel"]+1}_Pressure_Torr'] = []
        self.error_plot_data.clear()

        self.start_button.config(state=tk.DISABLED); self.manual_cal_button.config(state=tk.DISABLED)
        self.e_stop_button.config(state=tk.NORMAL)
        threading.Thread(target=self.run_calibration, daemon=True).start()

    def _predict_outlet_position(self, target_sp):
        """Predicts the best outlet position using interpolated historical data."""
        if not self.learned_outlet_positions: return None

        # Average the collected data for each setpoint
        avg_positions = {float(sp): sum(pos_list) / len(pos_list) for sp, pos_list in self.learned_outlet_positions.items() if pos_list}
        if not avg_positions: return None

        known_sps = sorted(avg_positions.keys())
        # If target is known, return the average
        if target_sp in known_sps: return avg_positions[target_sp]
        # If not enough data, return the closest known point
        if len(known_sps) < 2: return avg_positions[min(known_sps, key=lambda sp: abs(sp - target_sp))]

        # Linear interpolation/extrapolation for unknown points
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

    def run_calibration(self):
        """The main logic for the automated calibration sequence."""
        run_start_time = time.time()
        try:
            # Generate a composite list of setpoints from all active devices
            master_setpoints = set()
            for i in range(0, 101, 10): master_setpoints.add(round(self.standard_fs_value * i / 100, 2))
            for dut in self.active_duts:
                for i in range(0, 101, 10): master_setpoints.add(round(dut['fs'] * i / 100, 2))
            setpoints = sorted(list(master_setpoints))
            self.log_message(f"Generated composite setpoints: {setpoints}")

            self.after(0, self._setup_error_plot, setpoints)

            dut_specific_setpoints = {dut['channel']: {round(dut['fs'] * i / 100, 2) for i in range(0, 101, 10)} for dut in self.active_duts}

            for sp in setpoints:
                if not self.is_calibrating: break
                setpoint_start_time = time.time()
                predicted_pos = self._predict_outlet_position(sp)
                self.log_message(f"\n--- Setting {sp} Torr ---")
                self.state_controller.set_pressure(sp, predicted_outlet_pos=predicted_pos)
                self.log_message("Waiting for pressure to stabilize...")

                # --- Stability Check Loop ---
                stability_confirmed_time = None; out_of_tolerance_start_time = None
                relevant_duts = [d for d in self.active_duts if sp in dut_specific_setpoints.get(d['channel'], set())]
                priority_tolerance = min([d['fs'] * 0.005 for d in relevant_duts]) if relevant_duts else self.standard_fs_value * 0.005

                while self.is_calibrating:
                    if len(self.state_controller.pressure_history) < 10: time.sleep(0.5); continue

                    is_stable = statistics.stdev(self.state_controller.pressure_history) < (self.standard_fs_value * 0.0002)
                    stable_pressure = self.state_controller.current_pressure
                    if stable_pressure is None: time.sleep(1); continue

                    if is_stable:
                        if abs(stable_pressure - sp) <= priority_tolerance:
                            out_of_tolerance_start_time = None
                            if stability_confirmed_time is None: stability_confirmed_time = time.time()
                            # Require 3 seconds of continuous stability before proceeding
                            if (time.time() - stability_confirmed_time) >= 3.0:
                                self.log_message(f"  Pressure locked at {stable_pressure:.3f} Torr. Proceeding to log.")
                                break
                        else: # Stable, but out of tolerance
                            stability_confirmed_time = None
                            if out_of_tolerance_start_time is None:
                                self.log_message(f"  Pressure stable at {stable_pressure:.3f} Torr, but OUTSIDE tolerance (+/- {priority_tolerance:.4f} Torr).")
                                self.log_message("  Waiting 20 seconds before prompting...")
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

                if not self.is_calibrating: continue

                # --- Data Logging Phase ---
                self.log_message(f"  Starting 5s data log. Locking outlet valve.")
                self.state_controller.hold_outlet_valve = True
                log_start_time = time.time()
                standard_readings = []; dut_readings = {dut['channel']: [] for dut in self.active_duts}
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

                # --- Process and Save Logged Data ---
                mean_standard = np.mean(standard_readings) if standard_readings else np.nan
                if np.isnan(mean_standard): continue

                # Learn the outlet position from this successful setpoint
                current_outlet_pos = self.state_controller.outlet_valve_pos
                if current_outlet_pos is not None:
                    sp_key = round(sp, 3)
                    if sp_key not in self.learned_outlet_positions: self.learned_outlet_positions[sp_key] = []
                    self.learned_outlet_positions[sp_key].append(current_outlet_pos)
                    if len(self.learned_outlet_positions[sp_key]) > 10: self.learned_outlet_positions[sp_key] = self.learned_outlet_positions[sp_key][-10:]
                    self.log_message(f"  Updated history for {sp_key:.3f} Torr. Now have {len(self.learned_outlet_positions[sp_key])} data point(s).")

                self.data_storage['Setpoint_Torr'].append(sp)
                self.data_storage['Standard_Pressure_Torr'].append(mean_standard)
                log_line = f"  Logged -> Setpoint: {sp:.2f} | Standard (Avg): {mean_standard:.3f} Torr (took {time.time() - setpoint_start_time:.1f}s)"

                for dut in self.active_duts:
                    ch, fs = dut['channel'], dut['fs']
                    mean_dut = np.mean(dut_readings.get(ch, [])) if dut_readings.get(ch) else np.nan
                    self.data_storage[f'Device_{ch+1}_Pressure_Torr'].append(mean_dut)
                    if not np.isnan(mean_dut):
                        log_line += f" | Dev {ch+1} (Avg): {mean_dut:.3f} Torr"
                        if sp in dut_specific_setpoints.get(ch, set()):
                            error = mean_dut - mean_standard
                            if sp not in self.error_plot_data: self.error_plot_data[sp] = {}
                            self.error_plot_data[sp][ch] = {'error': error, 'time': time.time() - self.start_time}
                            if abs(error) > (fs * 0.005): self.log_message(f"  ⚠️ WARNING: Device {ch+1} OUTSIDE tolerance! Error: {error:+.4f} Torr")
                    else: log_line += f" | Dev {ch+1}: READ FAILED"
                self.log_message(log_line)
                self.after(0, self.update_error_plot) # Update plot from main thread

            if self.is_calibrating:
                self.log_message(f"\n--- Data Logging Complete in {time.time() - run_start_time:.1f} seconds. Saving data... ---")
                pd.DataFrame(self.data_storage).to_csv("calibration_results.csv", index=False)
                self.log_message("Data saved to 'calibration_results.csv'.")
                self._save_learned_data()

        except Exception as e:
            self.log_message(f"FATAL ERROR during logging: {e}")
        finally:
            if self.is_calibrating: self.after(0, self._generate_debug_plot)
            self.is_calibrating = False
            if self.state_controller: self.state_controller.close_valves()
            self.after(10, self.analyze_and_suggest_tuning)
            self.after(10, lambda: [self.start_button.config(state=tk.NORMAL), self.manual_cal_button.config(state=tk.NORMAL), self.e_stop_button.config(state=tk.DISABLED)])

    def _save_gui_config(self):
        """Saves the current GUI configuration to a JSON file."""
        config_data = {
            'inlet_com': self.inlet_com_var.get(),
            'outlet_com': self.outlet_com_var.get(),
            'daq_com': self.daq_com_var.get(),
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
        """Loads GUI configuration from a JSON file, with sane defaults."""
        try:
            with open(self.config_file, 'r') as f:
                config_data = json.load(f)
            self.log_message("Loaded GUI configuration from file.")
        except (FileNotFoundError, json.JSONDecodeError):
            self.log_message("No valid config file found, using defaults.")
            # Set default values if config file is not found or invalid
            config_data = {
                'inlet_com': 'COM5', 'outlet_com': 'COM6', 'daq_com': 'COM7',
                'std_fs': '100.0',
                'duts': [{'enabled': True, 'fs': '100.0'}] * 4
            }

        # Apply the loaded or default settings to the GUI variables
        self.inlet_com_var.set(config_data.get('inlet_com', ''))
        self.outlet_com_var.set(config_data.get('outlet_com', ''))
        self.daq_com_var.set(config_data.get('daq_com', ''))
        self.std_fs_var.set(config_data.get('std_fs', '100.0'))
        
        dut_configs = config_data.get('duts', [])
        for i, widget in enumerate(self.dut_widgets):
            if i < len(dut_configs):
                widget['enabled'].set(dut_configs[i].get('enabled', True))
                widget['fs'].set(dut_configs[i].get('fs', '100.0'))
            else: # If config is missing entries, use defaults
                widget['enabled'].set(True)
                widget['fs'].set('100.0')

    def _save_learned_data(self):
        """Saves the learned outlet positions to a JSON file."""
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
        """Creates and saves a detailed plot of the entire calibration run."""
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
        """Displays a new window with tuning suggestions after a run."""
        win = tk.Toplevel(self); win.title("Tuning Suggestions"); win.geometry("700x600")
        main_frame = tk.Frame(win, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        suggestion_box = scrolledtext.ScrolledText(main_frame, font=("Courier", 10), wrap=tk.WORD, relief=tk.SOLID, borderwidth=1)
        suggestion_box.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        suggestion_box.insert(tk.END, suggestions_text); suggestion_box.config(state=tk.DISABLED)
        tk.Button(main_frame, text="Close", command=win.destroy).pack()

    def analyze_and_suggest_tuning(self):
        """Analyzes calibration data and generates tuning suggestions for each DUT."""
        suggestion_parts = ["--- Post-Calibration Tuning Analysis ---"]
        any_suggestions = False

        for dut in self.active_duts:
            ch, fs = dut['channel'], dut['fs']
            std_points = np.array([s for s, d in zip(self.data_storage['Standard_Pressure_Torr'], self.data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
            dut_points = np.array([d for s, d in zip(self.data_storage['Standard_Pressure_Torr'], self.data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
            if len(dut_points) < 3: continue

            # Perform linear regression to find offset and span errors
            slope, intercept = np.polyfit(std_points, dut_points, 1)
            zero_offset_is_sig = abs(intercept) > (fs * 0.001)
            span_error_is_sig = abs(1.0 - slope) > 0.005

            # Calculate non-linearity
            non_linearity_points = [dut_p - (slope * std_p + intercept) for std_p, dut_p in zip(std_points, dut_points)]
            linearity_is_sig = False
            if non_linearity_points:
                max_non_linearity = max(np.abs(non_linearity_points))
                linearity_is_sig = max_non_linearity > (fs * 0.002)
                midpoint_error_from_line = non_linearity_points[np.abs(non_linearity_points).argmax()]

            if not (zero_offset_is_sig or span_error_is_sig or linearity_is_sig):
                suggestion_parts.append(f"\n--- Analysis for DUT {ch+1} ({fs} Torr FS) ---\n  ✅ SUCCESS: Device is well-calibrated.")
                continue

            any_suggestions = True
            suggestion_parts.append(f"\n--- Suggestions for DUT {ch+1} ({fs} Torr FS) ---")
            suggestion_parts.append(f"\n[ DIAGNOSIS ]\ny = {slope:.4f}x + {intercept:+.4f}")
            if zero_offset_is_sig: suggestion_parts.append(f" • ZERO OFFSET ERROR: {intercept:+.4f} Torr.")
            if span_error_is_sig: suggestion_parts.append(f" • SPAN (GAIN) ERROR: Gain is {'too high' if slope > 1 else 'too low'} (Slope={slope:.4f}).")
            if linearity_is_sig: suggestion_parts.append(f" • LINEARITY ERROR: Mid-range response {'bows UP' if midpoint_error_from_line > 0 else 'bows DOWN'}.")

            suggestion_parts.append("\n[ RECOMMENDED ADJUSTMENT PLAN ]\n\n1. ADJUST ZERO (0% FS)")
            if zero_offset_is_sig: suggestion_parts.append(f"   ➡️ ACTION: Adjust to {'LOWER' if intercept > 0 else 'RAISE'} the reading.")
            suggestion_parts.append("\n2. ADJUST SPAN (100% FS)")
            if span_error_is_sig: suggestion_parts.append(f"   ➡️ ACTION: Adjust to {'LOWER' if slope > 1 else 'RAISE'} the reading.")
            suggestion_parts.append("\n3. RE-CHECK ZERO (Critical Step)\n\n4. ADJUST LINEARITY (50% FS)")
            if linearity_is_sig: suggestion_parts.append(f"   ➡️ ACTION: Correct {'upward \"smiling\"' if midpoint_error_from_line > 0 else 'downward \"frowning\"'} bow.")

        final_suggestion_text = "\n".join(suggestion_parts)
        self.log_message(final_suggestion_text)
        if any_suggestions: self.show_tuning_suggestions_window(final_suggestion_text)

    def update_error_plot(self):
        """Updates the bar chart showing DUT error at each setpoint."""
        all_setpoints = sorted(self.error_plot_data.keys())
        if not all_setpoints: return
        self._setup_error_plot(all_setpoints)

        num_duts = len(self.active_duts)
        bar_height = min(np.diff(all_setpoints)) * 0.8 if len(all_setpoints) > 1 else 1.0
        bar_height = min(bar_height / num_duts, 2.0)

        for i, dut in enumerate(self.active_duts):
            ch = dut['channel']
            offset = (i - (num_duts - 1) / 2) * bar_height
            for sp, errors_data in self.error_plot_data.items():
                if ch in errors_data:
                    error_info = errors_data[ch]
                    self.ax_error.barh(sp + offset, error_info['error'], height=bar_height, color=self.dut_colors[ch], edgecolor='black', linewidth=0.5)
                    ha = 'left' if error_info['error'] >= 0 else 'right'
                    self.ax_error.text(error_info['error'], sp + offset, f" @{error_info['time']:.1f}s", va='center', ha=ha, fontsize=7, color='#555555')

        self.ax_error.relim(); self.ax_error.autoscale_view(scalex=True, scaley=False)
        self.canvas.draw_idle()

    def on_closing(self):
        """Handles the application closing event."""
        self._save_gui_config() # Save settings on close
        self._save_learned_data()
        self.is_calibrating = False; self.is_in_manual_mode = False

        if self.after_id: self.after_cancel(self.after_id)
        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()

        self.quit()
        self.destroy()

# =================================================================================
# Main Execution Block
# =================================================================================
if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()

