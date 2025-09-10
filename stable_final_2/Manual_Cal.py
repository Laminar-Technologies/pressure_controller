# -*- coding: utf-8 -*-
# ==============================================================================
# File:         Manual_Cal.py
# Author:       Gemini
# Date:         September 5, 2025
# Description:  This module provides the ManualCalFrame class, which contains
#               the UI and logic for the manual calibration mode. It allows
#               users to manually set pressure points, view focused plots,
#               and contributes to the system's learned position data.
#
# Version Update (Logic Fix):
#   - Removed the automatic call to set zero pressure on initialization.
#     This is now handled by the main GUI after turbo pump checks.
# ==============================================================================

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import statistics
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# =================================================================================
# Manual Calibration Frame Class
# =================================================================================
class ManualCalFrame(tk.Frame):
    """
    A tkinter Frame that encapsulates the entire manual calibration interface.

    This class is responsible for creating all the widgets for manual control,
    handling user input for setting pressure, managing the focused plot displays,
    and running a background thread to auto-learn stable setpoints.
    """
    def __init__(self, parent, main_app):
        """
        Initializes the manual calibration frame.

        Args:
            parent (tk.Widget): The parent widget (part of the main GUI).
            main_app (CalibrationGUI): The instance of the main application,
                used to access shared resources like the state controller.
        """
        super().__init__(parent)
        self.main_app = main_app
        self.state_controller = main_app.state_controller
        self.log_queue = main_app.log_queue

        # State variables for manual mode
        self.manual_learn_target = None
        self.last_manual_stability_state = False
        self.manual_point_learned = threading.Event()
        self._manual_learn_thread = None

        self.setup_ui()
        self.start_learning_thread()

    def setup_ui(self):
        """Creates and arranges all widgets for the manual calibration UI."""
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=3)

        # --- Left Control Panel ---
        left_panel = tk.Frame(self, padx=10, pady=10)
        left_panel.grid(row=0, column=0, sticky="nsew")

        tk.Label(left_panel, text="Manual Calibration", font=("Helvetica", 16, "bold")).pack(pady=5, anchor='w')
        tk.Label(left_panel, textvariable=self.main_app.live_pressure_var, font=("Helvetica", 20, "bold"), fg="#00529B").pack(pady=(0,10), anchor='w')

        # --- Setpoint Control Frame ---
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

        # --- Focus Control Frame ---
        device_list_frame = tk.LabelFrame(left_panel, text="Focus Control", padx=10, pady=10)
        device_list_frame.pack(pady=10, fill='both', expand=True, anchor='n')

        tk.Radiobutton(device_list_frame, text=f"Standard ({self.main_app.standard_fs_value} Torr FS)", variable=self.main_app.manual_focus_device,
                        value="std", command=self.on_manual_focus_change, anchor='w').pack(fill='x')

        for dut in self.main_app.active_duts:
            ch, fs = dut['channel'], dut['fs']
            tk.Radiobutton(device_list_frame, text=f"Focus on DUT {ch+1} ({fs} Torr FS)", variable=self.main_app.manual_focus_device,
                            value=f"ch{ch}", command=self.on_manual_focus_change, anchor='w').pack(fill='x')
        
        # --- Tuning Helper Frame ---
        self.tuning_helper_frame = tk.LabelFrame(left_panel, text="Tuning Helper", padx=10, pady=10)
        # self.tuning_helper_frame.pack(pady=10, fill='x', anchor='n') # Packed later in on_manual_focus_change

        tk.Label(self.tuning_helper_frame, text="Live Difference (DUT - Standard)").pack()
        self.diff_label = tk.Label(self.tuning_helper_frame, textvariable=self.main_app.manual_dut_diff_var, font=("Helvetica", 18, "bold"))
        self.diff_label.pack(pady=(0, 5))

        # Create the visual indicator plot
        self.diff_fig, self.ax_diff = plt.subplots(figsize=(3, 0.8), dpi=80)
        self.diff_fig.tight_layout(pad=0.8)
        self.diff_canvas = FigureCanvasTkAgg(self.diff_fig, master=self.tuning_helper_frame)
        self.diff_canvas.get_tk_widget().pack()

        # Initialize the plot elements that will be updated
        self.diff_bar = None
        self.trend_arrow = None
        self._setup_diff_indicator_plot()

        # --- Right Plot Panel ---
        self.manual_plot_panel = tk.Frame(self)
        self.manual_plot_panel.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Create Quadrant plot for Standard focus
        self.manual_quad_fig, self.axs_manual_quad = plt.subplots(2, 2, figsize=(9, 7), sharex=True, sharey=False)
        self.manual_quadrant_lines = []
        for i, ax in enumerate(self.axs_manual_quad.flat):
            ax.set_title(f'DUT {i+1} vs. Standard', fontsize=10)
            std_line, = ax.plot([], [], 'b-', label='Standard')
            dut_line, = ax.plot([], [], color=self.main_app.dut_colors[i], label=f'DUT {i+1}')
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
        # REMOVED: self.set_manual_pressure(0) - This is now handled by Main.py after turbo checks

    def _setup_diff_indicator_plot(self):
        """Initializes the matplotlib plot for the difference indicator bar."""
        self.ax_diff.clear()
        self.ax_diff.axvline(0, color='black', linestyle='-', linewidth=1.5)
        self.ax_diff.set_yticks([])
        self.ax_diff.set_xlabel("Error (Torr)", fontsize=8)
        self.ax_diff.tick_params(axis='x', labelsize=8)

        # Initialize placeholder artists that will be updated later
        self.diff_bar = self.ax_diff.barh(0, 0, height=0.5, color='grey', align='center')
        # Arrow is initialized but not visible
        self.trend_arrow = self.ax_diff.arrow(0, 0.4, 0, 0, head_width=0.1, head_length=0.05, fc='black', ec='black', length_includes_head=True)
        self.trend_arrow.set_visible(False)

    def update_diff_indicator(self, difference, history):
        """Redraws the difference indicator bar and trend arrow."""
        if np.isnan(difference) or self.main_app.manual_focus_channel is None:
            # Hide bar if no valid difference is available
            if self.diff_bar:
                self.diff_bar[0].set_width(0)
            if self.trend_arrow:
                self.trend_arrow.set_visible(False)
            self.diff_canvas.draw_idle()
            return

        # --- Update the Bar ---
        self.diff_bar[0].set_width(difference)

        # Update bar color based on error magnitude
        ch = self.main_app.manual_focus_channel
        fs = [d['fs'] for d in self.main_app.active_duts if d['channel'] == ch][0]
        tolerance = fs * 0.001  # A tight tolerance for visual cue
        
        if abs(difference) < tolerance:
            color = '#2ca02c'  # green
        elif abs(difference) < tolerance * 5:
            color = '#ff7f0e'  # orange
        else:
            color = '#d62728'  # red
        self.diff_bar[0].set_color(color)

        # --- Update Trend Arrow ---
        if len(history) > 5:
            # Simple trend: compare the average of the two halves of recent history
            first_half_avg = sum(history[:5]) / 5
            second_half_avg = sum(history[5:]) / len(history[5:])
            trend = second_half_avg - first_half_avg  # A negative trend is good (error decreasing)

            # Only show arrow if the trend is significant
            if abs(trend) > tolerance * 0.1:
                # Arrow points right for worsening trend, left for improving
                arrow_dx = 0.0001 if trend * np.sign(difference) > 0 else -0.0001
                self.trend_arrow.set_data(x=difference, y=0.45, dx=arrow_dx, dy=0)
                # Green for improving, red for worsening
                arrow_color = 'red' if np.abs(second_half_avg) > np.abs(first_half_avg) else 'green'
                self.trend_arrow.set_facecolor(arrow_color)
                self.trend_arrow.set_edgecolor(arrow_color)
                self.trend_arrow.set_visible(True)
            else:
                self.trend_arrow.set_visible(False)
        else:
            self.trend_arrow.set_visible(False)

        self.diff_canvas.draw_idle()

    def start_learning_thread(self):
        """Starts the background thread for auto-learning stable points."""
        if self._manual_learn_thread is None:
            self._manual_learn_thread = threading.Thread(target=self._run_manual_learning_loop, daemon=True)
            self._manual_learn_thread.start()

    def stop_learning_thread(self):
        """Signals the learning thread to stop (implicitly on destroy)."""
        if self._manual_learn_thread is not None:
             # The thread is a daemon, so it will exit when the app closes.
             # Joining is good practice if cleanup is needed.
            self._manual_learn_thread.join(timeout=0.5)

    def _run_manual_learning_loop(self):
        """
        Background thread to detect when the system is stable at a target
        setpoint and automatically learn the outlet valve position.
        """
        while self.main_app.is_in_manual_mode:
            time.sleep(0.5)
            # Wait until a target is set and it hasn't been learned yet.
            if self.manual_learn_target is None or self.manual_point_learned.is_set():
                continue

            history = self.state_controller.pressure_history
            if len(history) < history.maxlen:
                continue

            try:
                current_pressure = self.state_controller.current_pressure
                if current_pressure is None: continue

                # Define stability criteria
                is_stable = statistics.stdev(history) < (self.main_app.standard_fs_value * 0.0003)
                is_on_target = abs(current_pressure - self.manual_learn_target) < (self.main_app.standard_fs_value * 0.0015)
                is_stable_now = is_stable and is_on_target

                # When stability is first achieved, log it and learn the point.
                if is_stable_now and not self.last_manual_stability_state:
                    self.log_queue.put(f"** System has stabilized at {current_pressure:.3f} Torr. **")

                    sp = self.manual_learn_target
                    pos = self.state_controller.outlet_valve_pos

                    if pos is not None and sp > 0:
                        sp_key = round(sp, 3)
                        # Add the learned position to the main app's dictionary.
                        if sp_key not in self.main_app.learned_outlet_positions:
                            self.main_app.learned_outlet_positions[sp_key] = []

                        self.main_app.learned_outlet_positions[sp_key].append(pos)

                        # Keep only the last 10 learned points to adapt to drift.
                        if len(self.main_app.learned_outlet_positions[sp_key]) > 10:
                            self.main_app.learned_outlet_positions[sp_key] = self.main_app.learned_outlet_positions[sp_key][-10:]

                        self.log_queue.put(f"Auto-learned manual point for {sp_key:.3f} Torr. Now have {len(self.main_app.learned_outlet_positions[sp_key])} data point(s).")
                        self.manual_point_learned.set()

                self.last_manual_stability_state = is_stable_now
            except (statistics.StatisticsError, TypeError):
                continue

    def on_manual_focus_change(self):
        """
        Handles the user changing the focus device (Standard or a DUT).
        Switches between the quadrant plot and the single focused plot.
        """
        focus_id = self.main_app.manual_focus_device.get()
        # Clear trace history to restart the plot view.
        self.main_app.manual_trace_time.clear()
        self.main_app.manual_trace_std.clear()
        for i in range(4): self.main_app.manual_trace_duts[i].clear()

        if focus_id == 'std':
            self.main_app.manual_focus_channel = None
            self.log_queue.put(f"Manual control focus set to Standard (Quadrant View).")
            # Hide the tuning helper and single plot, show the quadrant plot.
            self.tuning_helper_frame.pack_forget()
            self.manual_single_canvas.get_tk_widget().pack_forget()
            self.manual_quad_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            # Only show plots for active DUTs.
            for i, plot_data in enumerate(self.manual_quadrant_lines):
                is_active = any(d['channel'] == i for d in self.main_app.active_duts)
                plot_data['ax'].set_visible(is_active)
            self.manual_quad_canvas.draw_idle()
        else:
            ch = int(focus_id.replace('ch',''))
            self.main_app.manual_focus_channel = ch
            fs = [d['fs'] for d in self.main_app.active_duts if d['channel'] == ch][0]
            self.log_queue.put(f"Manual control focus set to DUT {ch+1} ({fs} Torr).")
            # Show the tuning helper and single plot, hide the quadrant plot.
            self.tuning_helper_frame.pack(pady=10, fill='x', anchor='n')
            self.manual_quad_canvas.get_tk_widget().pack_forget()
            self.manual_single_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            # Configure the single plot for the selected DUT.
            self.ax_manual_single.set_title(f"Live Trace: DUT {ch+1} vs. Standard")
            self.manual_single_lines['dut'].set_color(self.main_app.dut_colors[ch])
            self.manual_single_lines['dut'].set_label(f'DUT {ch+1}')
            self.ax_manual_single.legend()
            self.manual_single_canvas.draw_idle()
            
            # Reconfigure the diff plot axis for the new FS
            # Set axis limits to +/- 0.5% of the DUT's full scale for a good visual range
            limit = fs * 0.005 
            self.ax_diff.set_xlim(-limit, limit)

    def _set_custom_pressure(self):
        """Sets a custom pressure value from the user entry field."""
        try:
            pressure = float(self.custom_pressure_entry.get())
            if not (0 <= pressure <= self.main_app.standard_fs_value):
                messagebox.showerror("Invalid Input", f"Pressure must be between 0 and {self.main_app.standard_fs_value} Torr.")
                return

            self.manual_learn_target = pressure
            self.manual_point_learned.clear()
            self.last_manual_stability_state = False

            predicted_pos = self.main_app._predict_outlet_position(pressure)
            threading.Thread(target=self.state_controller.set_pressure, args=(pressure, predicted_pos), daemon=True).start()
            self.manual_setpoint_var.set(f"Current Setpoint: {pressure:.3f} Torr")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for the pressure.")

    def set_manual_pressure(self, fs_fraction):
        """
        Sets pressure based on a fraction of the full-scale range of the
        currently focused device.
        """
        focus_id = self.main_app.manual_focus_device.get()
        target_fs = self.main_app.standard_fs_value

        # Use the DUT's FS range if one is focused.
        if focus_id != "std":
            ch = int(focus_id.replace('ch',''))
            target_fs = [d['fs'] for d in self.main_app.active_duts if d['channel'] == ch][0]

        pressure = target_fs * fs_fraction

        self.manual_learn_target = pressure
        self.manual_point_learned.clear()
        self.last_manual_stability_state = False

        predicted_pos = self.main_app._predict_outlet_position(pressure)
        # Use a thread to avoid freezing the GUI during the setpoint change.
        threading.Thread(target=self.state_controller.set_pressure, args=(pressure, predicted_pos), daemon=True).start()
        self.manual_setpoint_var.set(f"Current Setpoint: {pressure:.3f} Torr")

