# Main.py
import time
import queue
import json
import threading
import collections
import statistics
import os
import shutil
import glob

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
from PIL import Image, ImageTk

from DAQ_Controller import DAQController
from State_Machine_Controller import StateMachinePressureController
from Manual_Cal import ManualCalFrame
from Tuning_Analysis import generate_tuning_suggestions
from Auto_Cal_Logic import run_calibration
from Turbo_Controller import TurboController
from Cert_Generator import generate_certificate
from Asana_Imports.asana_logic import upload_cert_to_asana
from Asana_Imports.asana_api_client import AsanaClient
from MultiplexerController import MultiplexerController


class CalibrationGUI(tk.Tk):
    """
    The main application window for the calibration system.
    """
    def __init__(self):
        super().__init__()

        self.title("Multi-Device Adaptive Calibration System (Raspberry Pi Edition)")
        self.attributes('-zoomed', True)
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        self.state_controller = None
        self.daq = None
        self.turbo_controller = None
        self.multiplexer = None
        self.is_connected = False
        self.is_calibrating = False
        self.is_in_manual_mode = False
        self.start_time = 0
        self.after_id = None
        self.manual_frame = None
        self.e_stop_triggered_event = threading.Event()
        self.is_pumping_down = False

        self.standard_fs_ranges = {
            1: 0.1,
            2: 10.0,
            3: 1.0
        }

        self.active_duts = []
        self.active_duts_lock = threading.Lock()

        self.turbo_rpm_var = tk.StringVar(value="---- RPM")
        self.turbo_temp_var = tk.StringVar(value="--°C")
        self.dut_colors = ['#2ca02c', '#d62728', '#ffd700', '#8c564b']

        self.at_speed_color = '#00E676'
        self.starting_color = '#FFD600'
        self.stopped_color = '#a0a0a0'

        self.data_storage = {}
        self.error_plot_data = {}
        self.log_queue = queue.Queue()
        self.dut_pass_status = {}

        self.completed_duts = set()

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

        self.live_pressure_var = tk.StringVar(value="---.------ Torr")
        self.live_time_history = collections.deque(maxlen=500)
        self.live_std_pressure_history = collections.deque(maxlen=500)
        self.live_dut_pressure_history = {i: collections.deque(maxlen=500) for i in range(4)}

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

        self.main_focus_device = tk.StringVar(value="std")
        self.main_focus_channel = None
        self.focus_trace_std_plot = None
        self.focus_trace_dut_plot = None

        self.debug_full_time = []
        self.debug_full_std_pressure = []
        self.debug_full_dut_pressure = {i: [] for i in range(4)}
        self.debug_full_inlet_pos = []
        self.debug_full_outlet_pos = []

        self.setup_ui()
        self._load_gui_config()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after_id = self.after(250, self.periodic_update)

        self.generated_certs = []
        self.asana_client = None
        self.asana_config = None

    def get_serial_ports(self):
        """ Returns a list of available serial ports. """
        ports = []
        if os.name == 'posix':
            ports.extend(glob.glob('/dev/tty[A-Za-z]*'))
        elif os.name == 'nt':
            ports.extend([port.device for port in serial.tools.list_ports.comports()])
        return sorted(ports)


    def setup_ui(self):
        customers = sorted(["Customer A", "Customer B", "Customer C", "Applied Materials", "Laminar Tech"])
        tech_ids = sorted(["RW", "JG", "EM", "TC", "AB"])

        template_dir = 'templates'
        if not os.path.exists(template_dir):
            os.makedirs(template_dir)
        item_numbers = sorted([f.replace('.xlsx', '') for f in os.listdir(template_dir) if f.endswith('.xlsx')])

        top_frame = tk.Frame(self)
        top_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        top_frame.columnconfigure(0, weight=1)

        top_config_frame = tk.Frame(top_frame)
        top_config_frame.grid(row=0, column=0, sticky="ew")
        top_config_frame.columnconfigure(0, weight=2)
        top_config_frame.columnconfigure(1, weight=7)

        control_config_frame = tk.LabelFrame(top_config_frame, text="Control & Configuration", padx=10, pady=10)
        control_config_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        control_config_frame.columnconfigure(0, weight=1)
        control_config_frame.columnconfigure(1, weight=1)

        action_frame = tk.Frame(control_config_frame)
        action_frame.grid(row=0, column=0, sticky="n", padx=(0,10))

        self.connect_button = tk.Button(action_frame, text="Connect", command=self.toggle_connection, width=15)
        self.connect_button.pack(pady=2, fill=tk.X)

        self.manual_cal_button = tk.Button(action_frame, text="Manual Cal", command=self.toggle_manual_mode, state=tk.DISABLED, width=15)
        self.manual_cal_button.pack(pady=2, fill=tk.X)
        self.start_button = tk.Button(action_frame, text="Start Auto Cal", command=self.start_calibration_thread, state=tk.DISABLED, width=15)
        self.start_button.pack(pady=2, fill=tk.X)
        self.e_stop_button = tk.Button(action_frame, text="E-Stop", command=self.e_stop_action, bg="red", fg="white", state=tk.DISABLED, width=15)
        self.e_stop_button.pack(pady=2, fill=tk.X)
        self.resume_button = tk.Button(action_frame, text="Resume", command=self.resume_from_estop, bg="orange", state=tk.DISABLED, width=15)
        self.resume_button.pack(pady=2, fill=tk.X)

        self.upload_to_asana_button = tk.Button(action_frame, text="Upload to Asana", command=self.prompt_for_asana_upload, state=tk.DISABLED, width=15)
        self.upload_to_asana_button.pack(pady=2, fill=tk.X)

        config_frame = tk.Frame(control_config_frame)
        config_frame.grid(row=0, column=1, sticky="n")
        com_ports = self.get_serial_ports()
        valid_ranges = sorted([0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 500.0, 1000.0])

        tk.Label(config_frame, text="Inlet (Inv):").grid(row=0, column=0, sticky="e", pady=2)
        self.inlet_com_var = tk.StringVar(self)
        self.inlet_com_combo = ttk.Combobox(config_frame, textvariable=self.inlet_com_var, values=com_ports, width=12)
        self.inlet_com_combo.grid(row=0, column=1, sticky="w", pady=2)

        tk.Label(config_frame, text="Outlet (Dir):").grid(row=1, column=0, sticky="e", pady=2)
        self.outlet_com_var = tk.StringVar(self)
        self.outlet_com_combo = ttk.Combobox(config_frame, textvariable=self.outlet_com_var, values=com_ports, width=12)
        self.outlet_com_combo.grid(row=1, column=1, sticky="w", pady=2)

        tk.Label(config_frame, text="Turbo:").grid(row=2, column=0, sticky="e", pady=2)
        self.turbo_com_var = tk.StringVar(self)
        self.turbo_com_combo = ttk.Combobox(config_frame, textvariable=self.turbo_com_var, values=com_ports, width=12)
        self.turbo_com_combo.grid(row=2, column=1, sticky="w", pady=2)

        tk.Label(config_frame, text="System FS:").grid(row=4, column=0, sticky="e", pady=2)
        self.std_fs_var = tk.StringVar(self)
        tk.Label(config_frame, textvariable=self.std_fs_var, font=("Helvetica", 10, "bold")).grid(row=4, column=1, sticky="w", pady=2)

        tk.Label(config_frame, text="Tech ID:").grid(row=5, column=0, sticky="e", pady=2)
        self.tech_id_var = tk.StringVar(self)
        self.tech_id_combo = ttk.Combobox(config_frame, textvariable=self.tech_id_var, values=tech_ids, width=12)
        self.tech_id_combo.grid(row=5, column=1, sticky="w", pady=2)

        tk.Label(config_frame, text="Service Type:").grid(row=6, column=0, sticky="e", pady=2)
        self.service_type_var = tk.StringVar(self)
        self.service_type_combo = ttk.Combobox(config_frame, textvariable=self.service_type_var, width=12)
        self.service_type_combo.grid(row=6, column=1, sticky="w", pady=2)

        tk.Label(config_frame, text="STD ID:").grid(row=7, column=0, sticky="e", pady=2)
        self.std_id_var = tk.StringVar(self)
        self.std_id_combo = ttk.Combobox(config_frame, textvariable=self.std_id_var, width=12)
        self.std_id_combo.grid(row=7, column=1, sticky="w", pady=2)

        dut_frame = tk.LabelFrame(top_config_frame, text="Devices Under Test (DUTs)", padx=10, pady=10)
        dut_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 5))
        self.dut_widgets = []
        for i in range(4):
            dut_row_frame = tk.Frame(dut_frame)
            dut_row_frame.pack(fill=tk.X, expand=True, pady=1)

            dut_row_frame.columnconfigure(5, weight=2)
            dut_row_frame.columnconfigure(7, weight=2)
            dut_row_frame.columnconfigure(9, weight=2)
            dut_row_frame.columnconfigure(11, weight=3)
            dut_row_frame.columnconfigure(13, weight=3)
            
            tk.Label(dut_row_frame, text=f"Dev {i+1}:").grid(row=0, column=0, sticky='w')
            
            enabled_var = tk.BooleanVar(self)
            fs_var = tk.StringVar(self)
            
            check = tk.Checkbutton(dut_row_frame, text="En", variable=enabled_var)
            check.grid(row=0, column=1, padx=(2,5))
            
            tk.Label(dut_row_frame, text="FS:").grid(row=0, column=2)
            menu = tk.OptionMenu(dut_row_frame, fs_var, *valid_ranges)
            menu.grid(row=0, column=3, sticky='ew', padx=(0,5))

            model_var = tk.StringVar(self)
            serial_var = tk.StringVar(self)
            wip_var = tk.StringVar(self)
            customer_var = tk.StringVar(self)
            item_number_var = tk.StringVar(self)

            tk.Label(dut_row_frame, text="Model:").grid(row=0, column=4)
            model_entry = tk.Entry(dut_row_frame, textvariable=model_var)
            model_entry.grid(row=0, column=5, sticky='ew', padx=(0,5))

            tk.Label(dut_row_frame, text="Serial:").grid(row=0, column=6)
            serial_entry = tk.Entry(dut_row_frame, textvariable=serial_var)
            serial_entry.grid(row=0, column=7, sticky='ew', padx=(0,5))
            
            tk.Label(dut_row_frame, text="WIP:").grid(row=0, column=8)
            wip_entry = tk.Entry(dut_row_frame, textvariable=wip_var)
            wip_entry.grid(row=0, column=9, sticky='ew', padx=(0,5))
            
            tk.Label(dut_row_frame, text="Customer:").grid(row=0, column=10)
            customer_combo = ttk.Combobox(dut_row_frame, textvariable=customer_var, values=customers)
            customer_combo.grid(row=0, column=11, sticky='ew', padx=(0,5))

            tk.Label(dut_row_frame, text="Item #:").grid(row=0, column=12)
            item_combo = ttk.Combobox(dut_row_frame, textvariable=item_number_var, values=item_numbers)
            item_combo.grid(row=0, column=13, sticky='ew')

            self.dut_widgets.append({
                'enabled': enabled_var, 'fs': fs_var, 'check': check, 'menu': menu,
                'model': model_var, 'serial': serial_var, 'wip': wip_var,
                'customer': customer_var, 'item_number': item_number_var
            })

        top_status_frame = tk.Frame(top_frame)
        top_status_frame.grid(row=1, column=0, sticky="ew", pady=(5,0))
        top_status_frame.columnconfigure(0, weight=2)
        top_status_frame.columnconfigure(1, weight=2)
        top_status_frame.columnconfigure(2, weight=2)
        top_status_frame.columnconfigure(3, weight=3)
        top_status_frame.columnconfigure(4, weight=3)
        
        standard_select_frame = tk.LabelFrame(top_status_frame, text="Standard Selection", padx=10, pady=10)
        standard_select_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self.selected_standard = tk.IntVar(value=1)
        for channel, fs in self.standard_fs_ranges.items():
            ttk.Radiobutton(
                standard_select_frame,
                text=f"Std {channel} ({fs} Torr)",
                variable=self.selected_standard,
                value=channel,
                command=self.on_standard_change
            ).pack(anchor='w')

        turbo_status_frame = tk.LabelFrame(top_status_frame, text="Turbo Pump", padx=10, pady=10)
        turbo_status_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 5))
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

        focus_frame = tk.LabelFrame(top_status_frame, text="Focus Control", padx=10, pady=10)
        focus_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 5))
        self.focus_radio_std = tk.Radiobutton(focus_frame, text="Overall View", variable=self.main_focus_device,
                        value="std", command=self.on_main_focus_change, anchor='w')
        self.focus_radio_std.pack(fill='x')
        self.focus_radios_dut = []
        for i in range(4):
            rb = tk.Radiobutton(focus_frame, text=f"Focus DUT {i+1}", variable=self.main_focus_device,
                            value=f"ch{i}", command=self.on_main_focus_change, anchor='w', state=tk.DISABLED)
            rb.pack(fill='x')
            self.focus_radios_dut.append(rb)

        valve_status_frame = tk.LabelFrame(top_status_frame, text="Valve Control", padx=10, pady=10)
        valve_status_frame.grid(row=0, column=3, sticky="nsew", padx=(5, 5))
        valve_status_frame.columnconfigure(0, weight=1); valve_status_frame.columnconfigure(1, weight=1)

        outlet_open_frame = tk.Frame(valve_status_frame)
        outlet_open_frame.grid(row=0, column=0, pady=(0, 2))
        self.open_outlet_button = tk.Button(outlet_open_frame, text="Open", command=self._open_outlet_valve_confirmed, state=tk.DISABLED)
        self.open_outlet_button.grid(row=0, column=0)
        self.outlet_warning_label = tk.Label(outlet_open_frame, text="⚠️", fg="red", font=("Helvetica", 10, "bold"))
        self.outlet_warning_label.grid(row=0, column=1)

        outlet_fine_coarse_open_frame = tk.Frame(valve_status_frame)
        outlet_fine_coarse_open_frame.grid(row=1, column=0)
        self.fine_outlet_open_button = tk.Button(outlet_fine_coarse_open_frame, text="Fine+", command=self._fine_bump_outlet_open, state=tk.DISABLED)
        self.fine_outlet_open_button.grid(row=0, column=0)
        self.coarse_outlet_open_button = tk.Button(outlet_fine_coarse_open_frame, text="Coarse+", command=self._coarse_bump_outlet_open, state=tk.DISABLED)
        self.coarse_outlet_open_button.grid(row=0, column=1)

        tk.Label(valve_status_frame, text="Outlet Valve", font=("Helvetica", 10, "bold")).grid(row=2, column=0)
        self.outlet_pos_var = tk.StringVar(value="-- %")
        tk.Label(valve_status_frame, textvariable=self.outlet_pos_var, font=("Helvetica", 14, "bold")).grid(row=3, column=0)
        self.outlet_valve_fig, self.ax_outlet_valve = plt.subplots(figsize=(0.8, 0.8), dpi=80)
        self.outlet_valve_canvas = FigureCanvasTkAgg(self.outlet_valve_fig, master=valve_status_frame)
        self.outlet_valve_canvas.get_tk_widget().grid(row=4, column=0, pady=2)
        self._draw_valve(self.ax_outlet_valve, 0)

        outlet_close_frame = tk.Frame(valve_status_frame)
        outlet_close_frame.grid(row=5, column=0)
        self.fine_closed_button = tk.Button(outlet_close_frame, text="Fine-", command=self._fine_bump_outlet_closed, state=tk.DISABLED)
        self.fine_closed_button.grid(row=0, column=0)
        self.coarse_closed_button = tk.Button(outlet_close_frame, text="Coarse-", command=self._coarse_bump_outlet_closed, state=tk.DISABLED)
        self.coarse_closed_button.grid(row=0, column=1)
        self.close_valve_button = tk.Button(valve_status_frame, text="Close", command=self._close_outlet_valve, state=tk.DISABLED)
        self.close_valve_button.grid(row=6, column=0, pady=2)

        inlet_open_frame = tk.Frame(valve_status_frame)
        inlet_open_frame.grid(row=0, column=1, pady=(0, 2))
        self.open_inlet_button = tk.Button(inlet_open_frame, text="Open", command=self._open_inlet_valve_confirmed, state=tk.DISABLED)
        self.open_inlet_button.grid(row=0, column=0)
        self.inlet_warning_label = tk.Label(inlet_open_frame, text="⚠️", fg="red", font=("Helvetica", 10, "bold"))
        self.inlet_warning_label.grid(row=0, column=1)

        inlet_fine_coarse_open_frame = tk.Frame(valve_status_frame)
        inlet_fine_coarse_open_frame.grid(row=1, column=1)
        self.fine_inlet_open_button = tk.Button(inlet_fine_coarse_open_frame, text="Fine+", command=self._fine_bump_inlet_open, state=tk.DISABLED)
        self.fine_inlet_open_button.grid(row=0, column=0)
        self.coarse_inlet_open_button = tk.Button(inlet_fine_coarse_open_frame, text="Coarse+", command=self._coarse_bump_inlet_open, state=tk.DISABLED)
        self.coarse_inlet_open_button.grid(row=0, column=1)

        tk.Label(valve_status_frame, text="Inlet Valve", font=("Helvetica", 10, "bold")).grid(row=2, column=1)
        self.inlet_pos_var = tk.StringVar(value="-- %")
        tk.Label(valve_status_frame, textvariable=self.inlet_pos_var, font=("Helvetica", 14, "bold")).grid(row=3, column=1)
        self.inlet_valve_fig, self.ax_inlet_valve = plt.subplots(figsize=(0.8, 0.8), dpi=80)
        self.inlet_valve_canvas = FigureCanvasTkAgg(self.inlet_valve_fig, master=valve_status_frame)
        self.inlet_valve_canvas.get_tk_widget().grid(row=4, column=1, pady=2)
        self._draw_valve(self.ax_inlet_valve, 0)

        inlet_close_frame = tk.Frame(valve_status_frame)
        inlet_close_frame.grid(row=5, column=1)
        self.fine_inlet_closed_button = tk.Button(inlet_close_frame, text="Fine-", command=self._fine_bump_inlet_closed, state=tk.DISABLED)
        self.fine_inlet_closed_button.grid(row=0, column=0)
        self.coarse_inlet_closed_button = tk.Button(inlet_close_frame, text="Coarse-", command=self._coarse_bump_inlet_closed, state=tk.DISABLED)
        self.coarse_inlet_closed_button.grid(row=0, column=1)
        self.close_inlet_valve_button = tk.Button(valve_status_frame, text="Close", command=self._close_inlet_valve, state=tk.DISABLED)
        self.close_inlet_valve_button.grid(row=6, column=1, pady=2)

        pressure_readout_frame = tk.LabelFrame(top_status_frame, text="Live System Pressure", padx=10, pady=10)
        pressure_readout_frame.grid(row=0, column=4, sticky="nsew", padx=(5,0))
        tk.Label(pressure_readout_frame, textvariable=self.live_pressure_var, font=("Helvetica", 22, "bold")).pack(pady=(0,5))
        custom_sp_frame = tk.Frame(pressure_readout_frame)
        custom_sp_frame.pack(pady=5)
        tk.Label(custom_sp_frame, text="Setpoint (Torr):").pack(side=tk.LEFT)
        self.custom_pressure_entry_main = tk.Entry(custom_sp_frame, width=8)
        self.custom_pressure_entry_main.pack(side=tk.LEFT, padx=2)
        self.set_pressure_button = tk.Button(custom_sp_frame, text="Set", command=self._set_custom_pressure_main, state=tk.DISABLED, width=4)
        self.set_pressure_button.pack(side=tk.LEFT)
        system_action_frame = tk.Frame(pressure_readout_frame)
        system_action_frame.pack(pady=5)
        self.pump_button = tk.Button(system_action_frame, text="Pump to Vacuum", command=self._pump_to_vacuum, state=tk.DISABLED)
        self.pump_button.pack(side=tk.LEFT, padx=5)
        self.vent_button = tk.Button(system_action_frame, text="Vent System", command=self._vent_system, state=tk.DISABLED)
        self.vent_button.pack(side=tk.LEFT, padx=5)


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

    def on_standard_change(self):
        channel = self.selected_standard.get()
        standard_fs = self.standard_fs_ranges.get(channel, 1.0)
        self.std_fs_var.set(f"{standard_fs} Torr")
        self.standard_fs_value = standard_fs
        
        if self.multiplexer and self.is_connected:
            self.multiplexer.select_channel(channel)
            
            if standard_fs <= 1.0:
                self.multiplexer.set_range(0.01)
            elif standard_fs <= 100.0:
                self.multiplexer.set_range(0.1)
            else:
                self.multiplexer.set_range(1)
        else:
            pass

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_instruments()
        else:
            self.connect_instruments()

    def connect_instruments(self):
        self.log_message("Attempting to connect to all instruments...")
        self.connect_button.config(state=tk.DISABLED)
        threading.Thread(target=self._connect_thread, daemon=True).start()

    def disconnect_instruments(self):
        self.log_message("Disconnecting from all instruments...")
        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()
        if self.turbo_controller: self.turbo_controller.stop()
        if self.multiplexer: self.multiplexer.cleanup()

        self.is_connected = False
        self.connect_button.config(text="Connect", state=tk.NORMAL)
        self.set_config_state(tk.NORMAL)

        for btn in [self.start_button, self.manual_cal_button, self.e_stop_button, self.set_pressure_button, self.fine_outlet_open_button, self.fine_closed_button, self.coarse_outlet_open_button, self.coarse_closed_button, self.close_valve_button, self.open_outlet_button, self.open_inlet_button, self.fine_inlet_open_button, self.coarse_inlet_open_button, self.fine_inlet_closed_button, self.coarse_inlet_closed_button, self.pump_button, self.vent_button]:
            btn.config(state=tk.DISABLED)

        self.log_message("All instruments disconnected. Ready to reconfigure.")

    def _connect_thread(self):
        try:
            inlet_port = self.inlet_com_var.get()
            outlet_port = self.outlet_com_var.get()
            turbo_port = self.turbo_com_var.get()
            
            self.on_standard_change()
            
            self.state_controller = StateMachinePressureController(
                inlet_port=inlet_port,
                outlet_port=outlet_port,
                full_scale_pressure=self.standard_fs_value,
                log_queue=self.log_queue,
                e_stop_event=self.e_stop_triggered_event
            )
            self.log_message(f"Connected to Controllers on {inlet_port} & {outlet_port}.")

            self.daq = DAQController(host='<Raspberry Pi IP Address>', port=65432) # Replace with your Raspberry Pi's IP address
            self.log_message(f"Initialized DAQ controller.")

            self.turbo_controller = TurboController(turbo_port, self.log_queue)
            self.log_message(f"Connected to Turbo on {turbo_port}.")

            channel_pins = {1: 10, 2: 26, 3: 16}
            range_pins = {'x0.1': 21, 'x0.01': 24}
            self.multiplexer = MultiplexerController(channel_pins, range_pins, self.log_queue)
            self.on_standard_change()

            self.state_controller.start()
            self.turbo_controller.start()

            time.sleep(1.2)
            flags = self.turbo_controller.status_flags
            if not flags.get('is_on', False):
                self.log_message("Turbo pump is off. Sending start command...")
                self.turbo_controller.send_command("TMPON")
                time.sleep(1)

            if flags.get('standby', False):
                self.log_queue.put("Turbo in standby. Setting to nominal speed...")
                self.turbo_controller.send_command("NSP")
            else:
                self.log_queue.put("✅ Turbo pump is already at nominal speed.")

            self.after(0, self._on_connection_success)

        except (ValueError, ConnectionError, tk.TclError) as e:
            self.log_queue.put(f"ERROR: {e}")
            self.after(0, self._on_connection_failure)

    def _on_connection_success(self):
        self.is_connected = True
        self.connect_button.config(text="Disconnect", state=tk.NORMAL)
        self.e_stop_triggered_event.clear()

        with self.active_duts_lock:
            self.active_duts = [
                {
                    'channel': i, 'fs': float(w['fs'].get()),
                    'model': w['model'].get(), 'serial': w['serial'].get(),
                    'wip': w['wip'].get(), 'customer': w['customer'].get(),
                    'item_number': w['item_number'].get(),
                    'service_type': self.service_type_var.get(),
                    'cal_std_id': self.std_id_var.get()
                }
                for i, w in enumerate(self.dut_widgets) if w['enabled'].get()
            ]

        for i, rb in enumerate(self.focus_radios_dut):
            is_active = any(d['channel'] == i for d in self.active_duts)
            rb.config(state=tk.NORMAL if is_active else tk.DISABLED)

        fs_key = str(self.standard_fs_value)
        raw_data = self.learned_data.get(fs_key, {})
        self.learned_outlet_positions = {float(k): v for k, v in raw_data.items()}
        self.log_message(f"Activated learning profile for {fs_key} Torr FS ({len(self.learned_outlet_positions)} points).")

        self.start_time = time.time()

        for btn in [self.start_button, self.manual_cal_button, self.standby_button, self.nominal_button, self.start_pump_button, self.stop_pump_button, self.set_pressure_button, self.fine_outlet_open_button, self.fine_closed_button, self.coarse_outlet_open_button, self.coarse_closed_button, self.close_valve_button, self.open_outlet_button, self.open_inlet_button, self.fine_inlet_open_button, self.coarse_inlet_open_button, self.fine_inlet_closed_button, self.coarse_inlet_closed_button, self.pump_button, self.vent_button]:
            btn.config(state=tk.NORMAL)
        self.e_stop_button.config(state=tk.NORMAL)
        self.set_config_state(tk.DISABLED)
        self.configure_plots()

    def _on_connection_failure(self):
        self.log_message("Connection failed. Check ports and device power.")
        self.connect_button.config(state=tk.NORMAL)

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
            self.led_at_speed.config(bg='gray')
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

    def _activate_manual_override_cooldown(self):
        if self.state_controller and self.state_controller.is_connected:
            self.log_message("Manual override activated. Pausing adaptive logic for 15s.")
            self.state_controller.manual_override_active.set()
            self.after(15000, self._deactivate_manual_override_cooldown)

    def _deactivate_manual_override_cooldown(self):
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller.manual_override_active.clear()
            self.log_message("Manual override cooldown finished. Resuming adaptive logic.")

    def _fine_bump_outlet_open(self):
        self._activate_manual_override_cooldown()
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = min(self.state_controller.outlet_valve_pos + 0.1, 100.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped open to {new_pos:.2f}%")

    def _fine_bump_outlet_closed(self):
        self._activate_manual_override_cooldown()
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = max(self.state_controller.outlet_valve_pos - 0.1, 0.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped closed to {new_pos:.2f}%")

    def _coarse_bump_outlet_open(self):
        self._activate_manual_override_cooldown()
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = min(self.state_controller.outlet_valve_pos + 1.0, 100.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped open to {new_pos:.2f}%")

    def _coarse_bump_outlet_closed(self):
        self._activate_manual_override_cooldown()
        if self.state_controller and self.state_controller.is_connected and self.state_controller.outlet_valve_pos is not None:
            new_pos = max(self.state_controller.outlet_valve_pos - 1.0, 0.0)
            self.state_controller._write_to_outlet(f"S1 {new_pos:.2f}")
            self.state_controller._write_to_outlet("D1")
            self.log_message(f"BUMP -> Outlet bumped closed to {new_pos:.2f}%")

    def _close_outlet_valve(self):
        self._activate_manual_override_cooldown()
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller._write_to_outlet("C")
            self.log_message("BUMP -> Outlet valve closed.")

    def _fine_bump_inlet_open(self):
        self._activate_manual_override_cooldown()
        self._bump_inlet_valve(0.1)

    def _coarse_bump_inlet_open(self):
        self._activate_manual_override_cooldown()
        self._bump_inlet_valve(1.0)

    def _fine_bump_inlet_closed(self):
        self._activate_manual_override_cooldown()
        self._bump_inlet_valve(-0.1)

    def _coarse_bump_inlet_closed(self):
        self._activate_manual_override_cooldown()
        self._bump_inlet_valve(-1.0)

    def _close_inlet_valve(self):
        self._activate_manual_override_cooldown()
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller._write_to_inlet("C")
            self.log_message("BUMP -> Inlet valve commanded to close.")

    def _bump_inlet_valve(self, amount):
        if self.state_controller and self.state_controller.is_connected and self.state_controller.inlet_valve_pos is not None:
            current_pos = self.state_controller.inlet_valve_pos
            new_pos = max(0.0, min(100.0, current_pos - amount))
            self.state_controller._write_to_inlet(f"S5 {new_pos:.2f}")
            self.state_controller._write_to_inlet("D5")
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
            with open("Command_Reference.txt", 'r') as f:
                for line in f:
                    if line.strip().startswith("##"):
                        text_area.insert(tk.END, line.lstrip("# "), "header")
                    else:
                        text_area.insert(tk.END, line)
        except FileNotFoundError:
            text_area.insert(tk.INSERT, "Error: Command_Reference.txt not found.")

        text_area.config(state=tk.DISABLED)

        close_button = tk.Button(win, text="Close", command=win.destroy)
        close_button.pack(pady=5)
        win.transient(self)
        win.grab_set()

    def _draw_valve(self, ax, position_percent):
        if ax is None: return
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
        self._activate_manual_override_cooldown()
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
        self._activate_manual_override_cooldown()
        command = self.outlet_command_entry.get().strip()
        self.outlet_command_entry.delete(0, tk.END)
        if not command: return
        self.log_message(f"> OUTLET: {command}")
        if self.state_controller and self.state_controller.is_connected:
            self.state_controller._write_to_outlet(command)
            self.log_message("  Command sent.")
        else:
            self.log_message("Controller not connected.")

    def set_config_state(self, state):
        widgets_to_toggle = [
            self.inlet_com_combo, self.outlet_com_combo,
            self.turbo_com_combo, self.tech_id_combo,
            self.service_type_combo, self.std_id_combo
        ]
        for widget in widgets_to_toggle:
            try:
                widget.config(state=state)
            except tk.TclError:
                widget.configure(state=state)

        for widget_set in self.dut_widgets:
            widget_set['check'].config(state=state)
            widget_set['menu'].config(state=state)


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

        self.start_button.config(state=tk.DISABLED)
        self.manual_cal_button.config(state=tk.DISABLED)
        self.resume_button.config(state=tk.NORMAL)


    def resume_from_estop(self):
        self.log_message("Resuming operations...")
        self.e_stop_triggered_event.clear()

        self.start_button.config(state=tk.NORMAL)
        self.manual_cal_button.config(state=tk.NORMAL)
        self.resume_button.config(state=tk.DISABLED)

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

        self.on_main_focus_change()

        self.canvas.draw_idle()

    def _setup_error_plot(self, setpoints):
        self.ax_error.clear()
        self.ax_error.set_title("DUT Deviation")
        self.ax_error.set_xlabel("Error (Torr)")
        self.ax_error.set_ylabel("Setpoint (Torr)")

        if setpoints:
            min_sp = min(setpoints)
            max_sp = max(setpoints)
            padding = (max_sp - min_sp) * 0.1
            if padding < 1e-6:
                padding = self.standard_fs_value * 0.05 if hasattr(self, 'standard_fs_value') else 0.1

            self.ax_error.set_ylim(min_sp - padding, max_sp + padding)

        self.ax_error.axvline(0, color='k', linestyle='--', alpha=0.5)
        self.ax_error.grid(True, linestyle=':')

        with self.active_duts_lock:
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

                if self.e_stop_triggered_event.is_set():
                    self.led_starting.config(bg='gray')
                    self.led_at_speed.config(bg='gray')
                else:
                    self.led_starting.config(bg=self.starting_color if flags.get('accelerating', False) else 'gray')

                    if flags.get('decelerating', False):
                        blink_on = int(time.time() * 4) % 2 == 0
                        self.led_at_speed.config(bg=self.at_speed_color if blink_on else 'gray')
                    elif flags.get('at_speed', False):
                        self.led_at_speed.config(bg=self.at_speed_color)
                    else:
                        self.led_at_speed.config(bg='gray')

                self.led_fault.config(bg='red' if flags.get('fault', False) else 'gray')
                self.led_standby.config(bg=self.starting_color if flags.get('standby', False) else 'gray')


            if self.state_controller and self.state_controller.is_connected:
                std_pressure = self.state_controller.get_pressure() 
                inlet_pos = self.state_controller.inlet_valve_pos
                outlet_pos = self.state_controller.outlet_valve_pos
                
                if std_pressure is not None:
                    self.state_controller.current_pressure = std_pressure

                if outlet_pos is not None and outlet_pos > 1.0:
                    self.inlet_warning_label.grid_remove()
                else:
                    self.inlet_warning_label.grid(row=0, column=1)

                if std_pressure is not None and std_pressure > (self.standard_fs_value * 0.75):
                    self.outlet_warning_label.grid_remove()
                else:
                    self.outlet_warning_label.grid(row=0, column=1)


                self.live_pressure_var.set(f"{std_pressure:.6g} Torr" if std_pressure is not None else "---.------ Torr")

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

                with self.active_duts_lock:
                    local_active_duts = self.active_duts[:]

                for i in range(4):
                    dut_pressure = np.nan
                    if any(d['channel'] == i for d in local_active_duts):
                        voltage_from_daq = self.daq.read_voltage(i) if self.daq else None
                        if voltage_from_daq is not None:
                            fs_list = [d['fs'] for d in local_active_duts if d['channel'] == i]
                            if fs_list:
                                fs = fs_list[0]
                                voltage_divider_multiplier = 4.9
                                original_dut_voltage = voltage_from_daq * voltage_divider_multiplier
                                dut_pressure = original_dut_voltage * (fs / 10.0)


                    self.live_dut_pressure_history[i].append(dut_pressure)
                    if self.is_in_manual_mode:
                        self.manual_trace_duts[i].append(dut_pressure)
                    if self.is_calibrating:
                        self.debug_full_dut_pressure[i].append(dut_pressure)

                if not self.is_in_manual_mode and self.live_std_plot:
                    if len(self.live_time_history) == len(self.live_std_pressure_history):
                        self.live_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
                    else:
                        self.log_message("WARNING: Mismatch in std pressure plot lengths. Skipping update.")

                    for i, line in self.live_dut_plots.items():
                        is_active = any(d['channel'] == i for d in local_active_duts)
                        is_completed = i in self.completed_duts
                        line.set_visible(is_active and not is_completed)

                        if len(self.live_time_history) == len(self.live_dut_pressure_history[i]):
                            line.set_data(self.live_time_history, self.live_dut_pressure_history[i])
                        else:
                            self.log_message(f"WARNING: Mismatch in DUT {i+1} plot lengths. Skipping update.")

                    t_max_main = self.live_time_history[-1] if self.live_time_history else 0
                    t_min_main = max(0, t_max_main - 45)
                    self.ax_live_pressure.set_xlim(t_min_main, t_max_main + 1)

                    start_index = 0
                    for i, t in enumerate(self.live_time_history):
                        if t >= t_min_main:
                            start_index = i
                            break

                    visible_pressures = []
                    if self.live_std_pressure_history:
                        valid_std = [p for p in list(self.live_std_pressure_history)[start_index:] if p is not None and not np.isnan(p)]
                        if valid_std: visible_pressures.extend(valid_std)

                    for i, history in self.live_dut_pressure_history.items():
                        if any(d['channel'] == i for d in local_active_duts) and i not in self.completed_duts:
                            valid_dut = [p for p in list(history)[start_index:] if p is not None and not np.isnan(p)]
                            if valid_dut: visible_pressures.extend(valid_dut)

                    if visible_pressures:
                        min_p, max_p = min(visible_pressures), max(visible_pressures)
                        padding = (max_p - min_p) * 0.1 if max_p > min_p else self.standard_fs_value * 0.05
                        if padding == 0: padding = self.standard_fs_value * 0.05
                        self.ax_live_pressure.set_ylim(bottom=min_p - padding, top=max_p + padding)

                    if self.main_focus_channel is None:
                        pass
                    else: 
                        ch = self.main_focus_channel
                        if self.focus_trace_std_plot and self.focus_trace_dut_plot:
                            self.focus_trace_std_plot.set_data(self.live_time_history, self.live_std_pressure_history)
                            self.focus_trace_dut_plot.set_data(self.live_time_history, self.live_dut_pressure_history[ch])

                            t_max_focus = self.live_time_history[-1] if self.live_time_history else 0
                            t_min_focus = max(0, t_max_focus - 30)
                            self.ax_error.set_xlim(t_min_focus, t_max_focus + 1)

                            start_index_focus = 0
                            for i, t in enumerate(self.live_time_history):
                                if t >= t_min_focus:
                                    start_index_focus = i
                                    break

                            focus_visible_pressures = []
                            std_visible = [p for p in list(self.live_std_pressure_history)[start_index_focus:] if p is not None and not np.isnan(p)]
                            dut_visible = [p for p in list(self.live_dut_pressure_history[ch])[start_index_focus:] if p is not None and not np.isnan(p)]
                            if std_visible: focus_visible_pressures.extend(std_visible)
                            if dut_visible: focus_visible_pressures.extend(dut_visible)

                            if focus_visible_pressures:
                                min_p_focus = min(focus_visible_pressures)
                                max_p_focus = max(focus_visible_pressures)
                                padding_focus = (max_p_focus - min_p_focus) * 0.1
                                if padding_focus < 1e-6:
                                    padding_focus = self.standard_fs_value * 0.01
                                self.ax_error.set_ylim(bottom=min_p_focus - padding_focus, top=max_p_focus + padding_focus)

                    self.canvas.draw_idle()

                if self.is_in_manual_mode and self.manual_frame:
                    manual_visible_pressures = []
                    valid_manual_std = [p for p in self.manual_trace_std if p is not None and not np.isnan(p)]
                    if valid_manual_std: manual_visible_pressures.extend(valid_manual_std)

                    for i, history in self.manual_trace_duts.items():
                         if any(d['channel'] == i for d in local_active_duts):
                            valid_dut = [p for p in history if p is not None and not np.isnan(p)]
                            if valid_dut: manual_visible_pressures.extend(valid_dut)

                    y_bottom, y_top = self.ax_live_pressure.get_ylim()
                    if manual_visible_pressures:
                        min_p, max_p = min(manual_visible_pressures), max(manual_visible_pressures)
                        padding = (max_p * 0.05) + (self.standard_fs_value * 0.001)
                        y_bottom = min_p - padding
                        y_top = max_p + padding

                    t_max_manual = self.manual_trace_time[-1] if self.manual_trace_time else 0
                    if self.manual_focus_channel is None:
                        for i, plot_data in enumerate(self.manual_frame.manual_quadrant_lines):
                            ax = plot_data['ax']
                            if ax.get_visible():
                                plot_data['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                                plot_data['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[i])
                                ax.set_ylim(bottom=y_bottom, top=y_top)
                        self.manual_frame.axs_manual_quad.flat[0].set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                        self.manual_frame.manual_quad_canvas.draw_idle()
                    else:
                        ch = self.manual_focus_channel
                        self.manual_frame.manual_single_lines['std'].set_data(self.manual_trace_time, self.manual_trace_std)
                        self.manual_frame.manual_single_lines['dut'].set_data(self.manual_trace_time, self.manual_trace_duts[ch])
                        self.manual_frame.ax_manual_single.set_xlim(max(0, t_max_manual - 30), t_max_manual + 1)
                        self.manual_frame.ax_manual_single.set_ylim(bottom=y_bottom, top=y_top)
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

        self.after_id = self.after(250, self.periodic_update)

    def on_main_focus_change(self):
        """ Handles switching the right plot between overall error and single DUT trace. """
        focus_id = self.main_focus_device.get()
        if focus_id == 'std':
            self.main_focus_channel = None
            self._setup_error_plot(list(self.error_plot_data.keys()))
            self.update_error_plot()
        else:
            ch = int(focus_id.replace('ch',''))
            self.main_focus_channel = ch
            self._setup_focus_trace_plot()

    def _setup_focus_trace_plot(self):
        """ Configures the right-hand plot to show a live trace for a focused DUT. """
        self.ax_error.clear()
        ch = self.main_focus_channel
        if ch is None: return

        self.ax_error.set_title(f"DUT {ch+1} vs. Standard (Live)", fontsize=10)
        self.ax_error.set_xlabel("Time (s)", fontsize=8)
        self.ax_error.set_ylabel("Pressure (Torr)", fontsize=8)
        self.ax_error.tick_params(axis='both', labelsize=8)
        self.ax_error.grid(True, linestyle=':')

        self.focus_trace_std_plot, = self.ax_error.plot([], [], 'blue', linewidth=1.5, label='Standard')
        self.focus_trace_dut_plot, = self.ax_error.plot([], [], color=self.dut_colors[ch], linewidth=1.5, label=f'DUT {ch+1}')

        self.ax_error.legend(loc='upper left', fontsize='small')
        self.canvas.draw_idle()


    def prompt_for_asana_upload(self):
        if not self.generated_certs:
            messagebox.showinfo("No Certificates", "No certificates have been generated in this session.")
            return

        try:
            with open("Asana_Imports/config.json", 'r') as f:
                self.asana_config = json.load(f)

            self.asana_client = AsanaClient(
                token=self.asana_config["asana_token"],
                workspace_id=self.asana_config["workspace_id"]
            )
        except (FileNotFoundError, KeyError):
            messagebox.showerror("Config Error", "Could not load Asana configuration. Please check Asana_Imports/config.json.")
            return

        win = tk.Toplevel(self)
        win.title("Upload Certificates to Asana")
        win.geometry("400x300")

        tk.Label(win, text="Select certificates to upload:").pack(pady=10)

        cert_vars = {}
        for cert_path in self.generated_certs:
            var = tk.BooleanVar(value=True)
            wip = os.path.splitext(os.path.basename(cert_path))[0]
            tk.Checkbutton(win, text=f"{wip}.xlsx", variable=var).pack(anchor='w', padx=20)
            cert_vars[cert_path] = var

        def on_upload():
            for cert_path, var in cert_vars.items():
                if var.get():
                    wip = os.path.splitext(os.path.basename(cert_path))[0]
                    upload_cert_to_asana(wip, cert_path, self.asana_client, self.asana_config["gids"])
            win.destroy()

        ttk.Button(win, text="Upload Selected", command=on_upload).pack(pady=20)
        win.transient(self)
        win.grab_set()

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
            is_ready = flags.get('at_speed', False) and not flags.get('standby', False)

            if not is_ready:
                self.log_message(f"Waiting for turbo pump to be ready for {operation_name}...")

                if not flags.get('is_on', False):
                    self.log_message("Pump is off. Sending start command...")
                    self.turbo_controller.send_command("TMPON")
                    time.sleep(1)

                self.turbo_controller.send_command("NSP")

                while not self.e_stop_triggered_event.is_set():
                    flags = self.turbo_controller.status_flags
                    if flags.get('at_speed', False) and not flags.get('standby', False):
                        self.log_message(f"✅ Turbo pump is ready. Starting {operation_name}.")
                        return True

                    current_operation_active = self.is_calibrating or self.is_in_manual_mode or self.is_pumping_down
                    if not current_operation_active:
                        self.log_message(f"{operation_name} canceled by user.")
                        self.is_pumping_down = False
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
        with self.active_duts_lock:
            self.active_duts = [
                {
                    'channel': i, 'fs': float(w['fs'].get()),
                    'model': w['model'].get(), 'serial': w['serial'].get(),
                    'wip': w['wip'].get(), 'customer': w['customer'].get(),
                    'item_number': w['item_number'].get(),
                    'service_type': self.service_type_var.get(),
                    'cal_std_id': self.std_id_var.get()
                }
                for i, w in enumerate(self.dut_widgets) if w['enabled'].get()
            ]

        for dut in self.active_duts:
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
                return

        self.is_calibrating = True
        self.debug_full_time.clear(); self.debug_full_std_pressure.clear()
        self.debug_full_inlet_pos.clear(); self.debug_full_outlet_pos.clear()
        for i in range(4): self.debug_full_dut_pressure[i].clear()

        self.log_message("\n--- Starting Automated Data Logging ---")
        self.data_storage = {'Setpoint_Torr': [], 'Standard_Pressure_Torr': []}
        with self.active_duts_lock:
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
        current_service_types = list(self.service_type_combo['values'])
        new_service_type = self.service_type_var.get()
        if new_service_type and new_service_type not in current_service_types:
            current_service_types.append(new_service_type)

        current_std_ids = list(self.std_id_combo['values'])
        new_std_id = self.std_id_var.get()
        if new_std_id and new_std_id not in current_std_ids:
            current_std_ids.append(new_std_id)

        config_data = {
            'inlet_com': self.inlet_com_var.get(),
            'outlet_com': self.outlet_com_var.get(),
            'turbo_com': self.turbo_com_var.get(),
            'std_fs': self.std_fs_var.get(),
            'tech_id': self.tech_id_var.get(),
            'duts': [
                {
                    'enabled': w['enabled'].get(), 'fs': w['fs'].get(),
                    'model': '', 'serial': '', 'wip': '',
                    'customer': w['customer'].get(),
                    'item_number': w['item_number'].get()
                }
                for w in self.dut_widgets
            ],
            'service_types': sorted(current_service_types),
            'last_service_type': new_service_type,
            'std_ids': sorted(current_std_ids),
            'last_std_id': new_std_id
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
            config_data = {}

        self.inlet_com_var.set(config_data.get('inlet_com', ''))
        self.outlet_com_var.set(config_data.get('outlet_com', ''))
        self.turbo_com_var.set(config_data.get('turbo_com', ''))
        self.std_fs_var.set(config_data.get('std_fs', '100.0'))
        self.tech_id_var.set(config_data.get('tech_id', ''))

        service_types = config_data.get('service_types', ['Level 1 Repair'])
        self.service_type_combo['values'] = sorted(service_types)
        self.service_type_var.set(config_data.get('last_service_type', 'Level 1 Repair'))

        std_ids = config_data.get('std_ids', ['PVS 100'])
        self.std_id_combo['values'] = sorted(std_ids)
        self.std_id_var.set(config_data.get('last_std_id', 'PVS 100'))

        dut_configs = config_data.get('duts', [])
        for i, widget in enumerate(self.dut_widgets):
            if i < len(dut_configs):
                dut_conf = dut_configs[i]
                widget['enabled'].set(dut_conf.get('enabled', True))
                widget['fs'].set(dut_conf.get('fs', '100.0'))
                widget['customer'].set(dut_conf.get('customer', ''))
                widget['item_number'].set(dut_conf.get('item_number', ''))

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
            output_dir = "Analysis"
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, 'calibration_debug_trace.png')

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
            fig.suptitle('Full Calibration Run - Debug Trace', fontsize=16)

            ax1.set_title('Pressure vs. Time'); ax1.set_ylabel('Pressure (Torr)'); ax1.grid(True, linestyle=':')
            ax1.plot(self.debug_full_time, self.debug_full_std_pressure, label='Standard', color='blue', linewidth=2)
            with self.active_duts_lock:
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
            fig.savefig(output_path, dpi=150)
            plt.close(fig)
            self.log_message(f"Debug plot saved to '{output_path}'")
        except Exception as e:
            self.log_message(f"ERROR: Could not generate debug plot: {e}")

    def show_tuning_suggestions_window(self, suggestion_reports, plot_filenames):
        win = tk.Toplevel(self)
        win.title("Post-Calibration Analysis Report")
        win.geometry("800x800")

        main_frame = tk.Frame(win)
        main_frame.pack(fill=tk.BOTH, expand=1)

        my_canvas = tk.Canvas(main_frame)
        my_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

        scrollbar = ttk.Scrollbar(main_frame, orient=tk.VERTICAL, command=my_canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        my_canvas.configure(yscrollcommand=scrollbar.set)
        my_canvas.bind('<Configure>', lambda e: my_canvas.configure(scrollregion=my_canvas.bbox("all")))

        scrollable_frame = tk.Frame(my_canvas)
        my_canvas.create_window((0,0), window=scrollable_frame, anchor="nw")

        for ch, report_text in suggestion_reports.items():
            ttk.Separator(scrollable_frame, orient='horizontal').pack(fill='x', pady=10)

            report_label = tk.Label(scrollable_frame, text=report_text, font=("Courier", 10), justify=tk.LEFT)
            report_label.pack(pady=(5,10), padx=10, anchor="w")

            plot_file = plot_filenames.get(ch)
            if plot_file:
                try:
                    img = Image.open(plot_file)
                    img.thumbnail((600, 450), Image.LANCZOS)
                    photo = ImageTk.PhotoImage(img)
                    img_label = tk.Label(scrollable_frame, image=photo)
                    img_label.image = photo
                    img_label.pack(pady=10)
                except Exception as e:
                    self.log_message(f"Error loading plot for DUT {ch+1}: {e}")

    def analyze_and_suggest_tuning(self):
        with self.active_duts_lock:
            suggestion_reports, plot_filenames, self.dut_pass_status = generate_tuning_suggestions(self.data_storage, self.active_duts)

        if suggestion_reports:
            self.show_tuning_suggestions_window(suggestion_reports, plot_filenames)
        else:
            self.log_message("No analysis reports were generated.")

    def update_error_plot(self):
        all_setpoints = sorted(self.error_plot_data.keys())
        if not all_setpoints: return
        self._setup_error_plot(all_setpoints)

        with self.active_duts_lock:
            num_duts = len(self.active_duts)
            bar_height = min(np.diff(all_setpoints)) * 0.8 if len(all_setpoints) > 1 else 1.0
            bar_height = min(bar_height / num_duts, 2.0) if num_duts > 0 else 2.0

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

    def _pump_to_vacuum(self):
        self.is_pumping_down = True
        threading.Thread(target=self._pump_to_vacuum_thread, daemon=True).start()

    def _pump_to_vacuum_thread(self):
        if self._wait_for_turbo_ready("Pump to Vacuum"):
            if self.state_controller and not self.e_stop_triggered_event.is_set():
                self.log_message("Pumping system to vacuum...")
                self.state_controller.set_pressure(0)
        self.is_pumping_down = False

    def _vent_system(self):
        threading.Thread(target=self._vent_system_thread, daemon=True).start()

    def _vent_system_thread(self):
        if not self.state_controller: return
        self.log_message("VENT: Closing outlet valve...")
        self.state_controller._write_to_outlet("C")

        timeout = time.time() + 15
        while time.time() < timeout:
            if self.state_controller.outlet_valve_pos is not None and self.state_controller.outlet_valve_pos < 0.1:
                self.log_message("VENT: Outlet valve confirmed closed. Opening inlet.")
                self.state_controller._write_to_inlet("S5 0.0")
                self.state_controller._write_to_inlet("D5")
                return
            time.sleep(0.5)
        self.log_message("VENT ERROR: Timeout waiting for outlet valve to close. Vent aborted.")

    def _open_outlet_valve_confirmed(self):
        msg = ("DANGER: Opening the outlet valve fully can rapidly evacuate the system.\n\n"
               "This can damage sensitive components if the system is at high pressure.\n\n"
               "Are you sure you want to proceed?")
        if messagebox.askyesno("Confirm Outlet Valve Open", msg, icon='warning'):
            if self.state_controller:
                self.log_message("User confirmed: Opening outlet valve fully.")
                self.state_controller._write_to_outlet("S1 100.0")
                self.state_controller._write_to_outlet("D1")

    def _open_inlet_valve_confirmed(self):
        msg = ("DANGER: Opening the inlet valve fully will expose the system to the gas source pressure.\n\n"
               "If the outlet valve is open, this can damage the turbo pump.\n\n"
               "Are you sure you want to proceed?")
        if messagebox.askyesno("Confirm Inlet Valve Open", msg, icon='warning'):
            if self.state_controller:
                self.log_message("User confirmed: Opening inlet valve fully.")
                self.state_controller._write_to_inlet("S5 0.0")
                self.state_controller._write_to_inlet("D5")

    def on_closing(self):
        self._save_gui_config()
        self._save_learned_data()
        self.is_calibrating = False; self.is_in_manual_mode = False

        if self.after_id: self.after_cancel(self.after_id)

        if self.turbo_controller: self.turbo_controller.send_command("SBY")

        if self.state_controller: self.state_controller.close()
        if self.daq: self.daq.close()
        if self.turbo_controller: self.turbo_controller.stop()
        if self.multiplexer: self.multiplexer.cleanup()

        self.quit()
        self.destroy()

if __name__ == "__main__":
    app = CalibrationGUI()
    app.mainloop()