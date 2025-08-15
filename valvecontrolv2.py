# ==============================================================================
# Script Name: Baratron Pressure Controller Calibration Script
# Author: Gemini
# Date: August 12, 2025
# Description: This script automates the calibration of a Baratron pressure
#              transducer using an MKS 651 Series pressure controller. It
#              communicates with the controller via a serial (VISA) connection,
#              sets a series of pressure setpoints, waits for the pressure to
#              stabilize at each point, and logs the data to a CSV file.
#              This version includes a graphical user interface (GUI) with
#              real-time plots, a user-interactive terminal, and an E-Stop button.
#
# Prerequisites:
#   - Python 3
#   - pyvisa library (pip install pyvisa pyvisa-py)
#   - pandas library (pip install pandas)
#   - matplotlib library (pip install matplotlib)
#   - MKS 651C Pressure Controller connected via a serial-to-USB cable
#   - Correct VISA address for the serial port (e.g., 'ASRL/dev/ttyUSB0::INSTR')
# ==============================================================================

# --- Import necessary libraries ---
# 'pyvisa' is the library that lets our computer talk to lab instruments.
import pyvisa
# 'time' is a library for controlling time, like making the script pause.
import time
# 'pandas' is used for creating and saving data in a structured table format, like a spreadsheet.
import pandas as pd
# 're' is the regular expression library, used for finding specific text patterns.
import re
# 'matplotlib.pyplot' is the library we'll use for plotting the data.
import matplotlib.pyplot as plt
# This import allows us to embed matplotlib plots into a Tkinter window.
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# 'tkinter' is Python's standard GUI library.
import tkinter as tk
from tkinter import scrolledtext
# 'threading' allows us to run the calibration in the background so the GUI doesn't freeze.
import threading
# 'queue' is a thread-safe way to pass messages from the background thread to the GUI.
import queue

# =================================================================================
# BaratronController Class
# This is a blueprint for an object that can manage our MKS instruments.
# It holds all the functions (methods) needed to talk to the controller.
# =================================================================================

class BaratronController:
    """
    This class handles all communication with the MKS 651 Pressure Controller.
    Think of it as the brain that knows how to send and receive commands.
    """
    # This dictionary maps the pressure values (in Torr) to their corresponding
    # integer codes as specified by the MKS 651C manual. This is used for
    # setting the pressure range of the device.
    RANGE_CODES = {
        0.1: 0, 0.2: 1, 0.5: 2, 1.0: 3, 2.0: 4, 5.0: 5, 10.0: 6, 50.0: 7,
        100.0: 8, 500.0: 9, 1000.0: 10, 5000.0: 11, 10000.0: 12, 1.33: 13,
        2.66: 14, 13.33: 15, 133.3: 16, 1333.0: 17, 6666.0: 18, 13332.0: 19
    }

    def __init__(self, pressure_visa_address, full_scale_pressure):
        """
        This is the constructor, which runs when we create a new 'BaratronController' object.
        It sets up the connection to the physical device.

        Args:
            pressure_visa_address (str): The unique address for the instrument on the computer.
                                         For a USB-serial port on Linux, it often looks like
                                         'ASRL/dev/ttyUSB0::INSTR'.
            full_scale_pressure (float): The maximum pressure the transducer can measure, in Torr.
                                         This is important for calculations.
        """
        # Create a resource manager to find and open the instrument.
        self.rm = pyvisa.ResourceManager()
        
        # Open the connection to the MKS 651 Pressure Controller using its address.
        self.pressure_controller = self.rm.open_resource(pressure_visa_address)
        
        # Set the communication settings for the connection. These must match
        # the settings on the MKS 651 controller itself.
        self.pressure_controller.timeout = 5000  # Give it up to 5 seconds to respond.
        self.pressure_controller.baud_rate = 9600 # The speed of data transfer.
        self.pressure_controller.data_bits = 8    # How many bits per data packet.
        self.pressure_controller.parity = pyvisa.constants.Parity.none
        self.pressure_controller.stop_bits = pyvisa.constants.StopBits.one
        
        # These lines tell the script what character signals the end of a command
        # or a response from the controller. The MKS manual specifies '\r' (carriage return).
        self.pressure_controller.read_termination = '\r'
        self.pressure_controller.write_termination = '\r'
        
        # Save the full-scale pressure value for later calculations.
        self.full_scale_pressure = full_scale_pressure
        
        # A simple flag to check if the connection is active.
        self.is_connected = True
        

    def write_command(self, command):
        """
        Sends a command to the instrument without waiting for a response.
        This is used for commands like "set this pressure" or "open the valve."
        """
        try:
            self.pressure_controller.write(command)
        except pyvisa.VisaIOError as e:
            raise e # Re-raise for the GUI to handle.

    def query_command(self, command):
        """
        Sends a command and waits for a response from the instrument.
        This is used for commands like "what is the current pressure?"
        """
        try:
            response = self.pressure_controller.query(command)
            return response
        except pyvisa.VisaIOError as e:
            raise e
            return None

    def set_pressure(self, pressure):
        """
        Tells the MKS controller to achieve a specific target pressure.
        
        Args:
            pressure (float): The desired pressure in Torr.
        """
        # The MKS 651C controller expects the pressure as a percentage of its full scale.
        # We must convert our target pressure (e.g., 50 Torr) into this percentage.
        setpoint_percentage = (pressure / self.full_scale_pressure) * 100
        
        # The command format is 'S1 [percentage]'. The :.2f formats the number
        # to two decimal places.
        setpoint_command = f"S1 {setpoint_percentage:.2f}"
        self.write_command(setpoint_command)

        # The 'D1' command activates the setpoint, telling the controller to
        # start trying to reach that pressure.
        self.write_command("D1")
        
    def set_pressure_range(self, range_value):
        """
        Sets the pressure range of the MKS 651C using the 'E' command and the
        corresponding integer code.
        
        Args:
            range_value (float): The full-scale pressure range to set in Torr.
        """
        # Check if the provided range_value exists in our predefined map.
        if range_value in self.RANGE_CODES:
            code = self.RANGE_CODES[range_value]
            # --- CRITICAL FIX: Removed the space between 'E' and the code. ---
            range_command = f"E{code}"
            self.write_command(range_command)
        else:
            raise ValueError(f"Warning: The provided range value {range_value} is not a valid preset.")

    def get_pressure(self):
        """
        Asks the MKS controller for the current pressure reading and
        converts it into the correct units (Torr).

        Returns:
            float: The current pressure in Torr, or None if the reading fails.
        """
        # The 'R5' command asks the controller for its current pressure value.
        response = self.query_command("R5")
        if response:
            try:
                # Use a regular expression to find the numeric part of the response.
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    # The controller sends back a number which is a percentage of full scale.
                    percentage_reading = float(match.group())
                    pressure = (percentage_reading / 100) * self.full_scale_pressure
                    return pressure
                else:
                    return None
            except ValueError as e:
                return None
        return None
    
    def get_valve_position(self):
        """
        Asks the MKS controller for the current valve output percentage.
        This represents the valve's position (e.g., 0 = closed, 100 = full open).

        Returns:
            float: The current valve output in percentage (0-100), or None if reading fails.
        """
        # The 'R6' command asks the controller for its current valve output percentage.
        response = self.query_command("R6")
        if response:
            try:
                # Use a regular expression to find the numeric part of the response.
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    valve_position = float(match.group())
                    return valve_position
                else:
                    return None
            except ValueError as e:
                return None
        return None
        
    def close_valve(self):
        """
        This function sends the command to close the pressure control valve.
        """
        self.write_command("C")
            
    def close(self):
        """
        This function safely closes the connection to the instrument.
        """
        if self.is_connected:
            self.pressure_controller.close()
            self.is_connected = False
        
# =================================================================================
# BaratronGUI Class
# This class handles all the GUI logic and application flow.
# =================================================================================
class BaratronGUI(tk.Tk):
    def __init__(self, pressure_visa_address):
        """
        Initializes the main application window and its components.
        """
        super().__init__()
        
        self.title("MKS 651C Calibration & Control")
        # Configure the main window to be resizable, with the middle row
        # expanding to fill any extra space.
        self.rowconfigure(0, weight=0) # Top frame won't expand
        self.rowconfigure(1, weight=1) # Plot frame will expand
        self.rowconfigure(2, weight=0) # Bottom frame won't expand
        self.columnconfigure(0, weight=1) # Main column will expand

        self.pressure_visa_address = pressure_visa_address
        self.full_scale_value = None
        self.baratron = None
        self.is_calibrating = False
        
        self.setpoint_data = []
        self.measured_data = []
        self.live_pressure_history = []
        self.live_valve_position_history = []
        self.live_time_history = []
        self.start_global_time = 0.0
        
        # Create a thread-safe queue for logging messages from the background thread.
        self.log_queue = queue.Queue()
        # This thread will handle the calibration and data acquisition.
        self.calibration_thread = None
        
        self.setup_ui()
            
        # This protocol handles closing the window gracefully.
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Start the periodic GUI update check.
        self.after(100, self.periodic_update)

    def setup_ui(self):
        """
        Configures the main window layout using the grid manager.
        """
        # Control frame at the top for connection and full-scale input.
        # Placed in row 0, column 0.
        top_control_frame = tk.Frame(self)
        top_control_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        
        # --- NEW DROPDOWN MENU FOR FULL-SCALE PRESSURE ---
        fs_label = tk.Label(top_control_frame, text="Full-Scale Torr:")
        fs_label.pack(side=tk.LEFT, padx=(0, 5))
        
        # Get the valid pressure ranges from the BaratronController class
        valid_ranges = sorted(BaratronController.RANGE_CODES.keys())
        self.fs_variable = tk.StringVar(self)
        self.fs_variable.set(valid_ranges[9]) # Default to 500.0 Torr
        self.fs_dropdown = tk.OptionMenu(top_control_frame, self.fs_variable, *valid_ranges)
        self.fs_dropdown.pack(side=tk.LEFT, padx=(0, 10))
        
        connect_button = tk.Button(top_control_frame, text="Connect", command=self.connect_instrument)
        connect_button.pack(side=tk.LEFT)

        # Add a button to start the calibration.
        self.start_button = tk.Button(top_control_frame, text="Start Calibration", command=self.start_calibration_thread, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT, padx=(10, 0))
        
        # Add a new E-Stop button.
        self.e_stop_button = tk.Button(top_control_frame, text="E-Stop", command=self.e_stop_action, bg="red", fg="white", state=tk.DISABLED)
        self.e_stop_button.pack(side=tk.LEFT, padx=(10, 0))
        
        # The main plot frame, which will expand to fill the available space.
        # Placed in row 1, column 0.
        plot_frame = tk.Frame(self)
        plot_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        
        # The control frame for the terminal, placed at the bottom.
        # Placed in row 2, column 0.
        control_frame = tk.Frame(self)
        control_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        
        # --- Setup the Matplotlib plots ---
        # Create a single figure with two subplots, stacked vertically.
        # Increased figsize to provide more room.
        self.fig, (self.ax_cal, self.ax_live) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Use explicit subplot adjustments instead of tight_layout()
        self.fig.subplots_adjust(
            left=0.1,    # More room on the left for the Y-axis label
            right=0.9,   # More room on the right for the second Y-axis label
            bottom=0.1,  # More room on the bottom for the X-axis label
            top=0.9,     # More room on the top for the title
            hspace=0.5,  # Increased vertical space between subplots
            wspace=0.2   # Horizontal space between subplots (if needed, not used here)
        )
        
        # Create a Tkinter canvas to hold the matplotlib figure.
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # --- Setup the terminal UI ---
        terminal_label = tk.Label(control_frame, text="Terminal Output", font=("Courier", 12))
        terminal_label.pack(side=tk.TOP, pady=(5,0))
        
        # Set a reasonable height for the scrolled text area so it doesn't take over.
        self.terminal_text = scrolledtext.ScrolledText(control_frame, wrap=tk.WORD, height=10, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00")
        self.terminal_text.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.terminal_text.config(state=tk.DISABLED) # Disable editing
        
        input_frame = tk.Frame(control_frame)
        input_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=(0, 5))
        
        input_label = tk.Label(input_frame, text="Command:", font=("Courier", 10))
        input_label.pack(side=tk.LEFT)
        
        self.command_entry = tk.Entry(input_frame, font=("Courier", 10), bg="#2c2c2c", fg="#00ff00", insertbackground="#00ff00")
        self.command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        # Bind the Enter key to the send_command method.
        self.command_entry.bind("<Return>", self.send_command)
        
    def connect_instrument(self):
        """
        Handles the connection to the instrument and initializes the plots.
        """
        try:
            # Get the full-scale value from the dropdown menu
            self.full_scale_value = float(self.fs_variable.get())
                
            self.baratron = BaratronController(self.pressure_visa_address, full_scale_pressure=self.full_scale_value)
            self.log_message("Successfully connected to the MKS 651 Pressure Controller.")
            self.start_button.config(state=tk.NORMAL)
            self.e_stop_button.config(state=tk.NORMAL)
            self.fs_dropdown.config(state=tk.DISABLED)

            self.configure_plots()
        except ValueError as e:
            self.log_message(f"Invalid input: {e}")
        except Exception as e:
            self.log_message(f"Error connecting to the instrument: {e}")
            self.log_message("Please check the VISA address and physical connection.")

    def e_stop_action(self):
        """
        Immediately stops the calibration and closes the valve.
        This is a crucial safety function.
        """
        self.log_message("\n*** EMERGENCY STOP ACTIVATED ***")
        self.is_calibrating = False # This flag will stop the background loop
        self.log_message("Calibration process stopped.")
        
        # Immediately close the valve to stop pressure changes.
        if self.baratron and self.baratron.is_connected:
            self.baratron.close_valve()
            self.log_message("Pressure control valve closed.")
        else:
            self.log_message("Instrument not connected. Cannot close valve.")
            
        # Reset the GUI state
        self.start_button.config(state=tk.NORMAL)
        self.e_stop_button.config(state=tk.DISABLED)
        self.fs_dropdown.config(state=tk.NORMAL)
        
    def configure_plots(self):
        """
        Configures the plot axes once the full-scale value is known.
        """
        # Configure the top subplot for the main calibration curve.
        self.ax_cal.set_title("Baratron Calibration Curve")
        self.ax_cal.set_xlabel("Setpoint Pressure (Torr)")
        self.ax_cal.set_ylabel("Measured Pressure (Torr)")
        self.ax_cal.grid(True)
        self.ax_cal.set_xlim([0, self.full_scale_value])
        self.ax_cal.set_ylim([0, self.full_scale_value])
        self.ax_cal.plot([0, self.full_scale_value], [0, self.full_scale_value], 'r--', label='Ideal')
        self.live_cal_plot, = self.ax_cal.plot([], [], 'bo-', label='Measured')
        self.ax_cal.legend()
        
        # Configure the bottom subplot for the live pressure trace.
        self.ax_live.set_title("Live Pressure and Valve Position Trace (Last 90s)")
        self.ax_live.set_xlabel("Time (s)")
        self.ax_live.set_ylabel("Valve Position (%)", color='b') # Valve position on the left
        self.ax_live.grid(True)
        self.live_valve_plot, = self.ax_live.plot([], [], 'b--', label='Valve Position') # Blue dashed line
        self.ax_live.set_ylim([-5, 105])
        
        # Create a second y-axis for the pressure trace on the right side.
        self.ax_live_valve = self.ax_live.twinx()
        self.ax_live_valve.set_ylabel("Pressure (Torr)", color='g') # Pressure on the right
        self.ax_live_valve.tick_params(axis='y', labelcolor='g')
        self.ax_live_valve.set_ylim([-5, self.full_scale_value * 1.5])
        self.live_pressure_plot, = self.ax_live_valve.plot([], [], 'g-', label='Pressure') # Green solid line
        
        # Add a combined legend
        lines, labels = self.ax_live.get_legend_handles_labels()
        lines2, labels2 = self.ax_live_valve.get_legend_handles_labels()
        self.ax_live_valve.legend(lines + lines2, labels + labels2, loc='upper left')

        self.canvas.draw_idle()

    def log_message(self, message):
        """
        Logs a message to the terminal text area. This method is thread-safe.
        """
        self.log_queue.put(message)

    def periodic_update(self):
        """
        A function that runs periodically to update the GUI with new data and logs.
        """
        # Process messages from the log queue.
        while not self.log_queue.empty():
            message = self.log_queue.get()
            self.terminal_text.config(state=tk.NORMAL)
            self.terminal_text.insert(tk.END, f"\n{message}")
            self.terminal_text.see(tk.END)
            self.terminal_text.config(state=tk.DISABLED)
            
        # Update the live plots if we are calibrating.
        if self.is_calibrating:
            current_time = self.live_time_history[-1] if self.live_time_history else 0
            self.ax_live.set_xlim(max(0, current_time - 90), current_time + 1)
            self.live_pressure_plot.set_data(self.live_time_history, self.live_pressure_history)
            self.live_valve_plot.set_data(self.live_time_history, self.live_valve_position_history)
            
            # These calls are critical for preventing text from overlapping on dynamic plots.
            self.ax_live.relim()
            self.ax_live.autoscale_view()
            self.canvas.draw_idle()
        
        self.after(100, self.periodic_update)

    def start_calibration_thread(self):
        """
        Starts the calibration process in a new, non-blocking thread.
        """
        if self.baratron and self.baratron.is_connected and not self.is_calibrating:
            self.log_message("Starting calibration process...")
            self.is_calibrating = True
            # Reset history for a new run
            self.setpoint_data = []
            self.measured_data = []
            self.live_pressure_history = []
            self.live_valve_position_history = []
            self.live_time_history = []
            self.start_global_time = time.time()
            self.live_cal_plot.set_data([], [])
            
            # The target function for our new thread is run_calibration.
            self.calibration_thread = threading.Thread(target=self.run_calibration)
            self.calibration_thread.daemon = True # Allows the thread to exit with the main program.
            self.calibration_thread.start()
            
            # Disable start button and enable E-stop
            self.start_button.config(state=tk.DISABLED)
            self.e_stop_button.config(state=tk.NORMAL)
        else:
            self.log_message("Cannot start calibration. Please check connection or if calibration is already running.")

    def run_calibration(self):
        """
        The main calibration logic. This runs in a separate thread.
        """
        try:
            # Create a list of all the pressure setpoints we want to calibrate against.
            setpoints = [round(self.full_scale_value * i / 100, 2) for i in range(0, 101, 10)]
            
            self.log_message("Enabling remote control...")
            self.baratron.write_command("RE 1")
            time.sleep(1)

            self.log_message(f"Setting pressure range to {self.full_scale_value} Torr...")
            self.baratron.set_pressure_range(self.full_scale_value)
            time.sleep(1) # Give the controller a moment to process the command
            
            # Define the tolerances for stabilization and measurement based on user input.
            measurement_tolerance = self.full_scale_value * 0.0005 # +/- 0.05% for averaging reads

            for sp in setpoints:
                # Check the flag here to allow the E-Stop to interrupt the loop.
                if not self.is_calibrating:
                    self.log_message("Calibration was manually stopped.")
                    break
                    
                self.log_message(f"\n--- Starting calibration for {sp} Torr ---")
                
                self.baratron.set_pressure(sp)
                
                # --- Stabilization Phase ---
                self.log_message(f"Waiting for pressure to stabilize at {sp} Torr...")
                stabilized = False
                
                # Special case for the zero setpoint
                if sp == 0:
                    self.log_message("Handling zero setpoint. Waiting for pressure to stop decreasing.")
                    start_stable_time = None
                    last_pressure = float('inf')
                    while not stabilized:
                        if not self.is_calibrating: break
                        
                        current_pressure = self.baratron.get_pressure()
                        current_valve_position = self.baratron.get_valve_position()
                        
                        if current_pressure is not None and current_valve_position is not None:
                            self.live_time_history.append(time.time() - self.start_global_time)
                            self.live_pressure_history.append(current_pressure)
                            self.live_valve_position_history.append(current_valve_position)
                        
                            if current_pressure >= last_pressure - 0.0001: # Small noise margin
                                if start_stable_time is None:
                                    start_stable_time = time.time()
                            else:
                                start_stable_time = None # Reset if pressure continues to drop
                                
                            last_pressure = current_pressure
                            
                            if start_stable_time is not None and (time.time() - start_stable_time) >= 5.0:
                                self.log_message("Pressure has reached its lowest point and stabilized for 5s. Beginning 10s measurement.")
                                stabilized = True
                                
                        time.sleep(0.1)
                else:
                    # Logic for non-zero setpoints (requiring continuous stability).
                    start_stable_time = None
                    self.log_message("Waiting for continuous stability for 5 seconds...")
                    
                    while not stabilized:
                        if not self.is_calibrating: break
                        
                        current_pressure = self.baratron.get_pressure()
                        current_valve_position = self.baratron.get_valve_position()
                        
                        if current_pressure is not None and current_valve_position is not None:
                            self.live_time_history.append(time.time() - self.start_global_time)
                            self.live_pressure_history.append(current_pressure)
                            self.live_valve_position_history.append(current_valve_position)
                        
                            if abs(current_pressure - sp) <= measurement_tolerance:
                                if start_stable_time is None:
                                    start_stable_time = time.time()
                            else:
                                start_stable_time = None
                                
                            if start_stable_time is not None and (time.time() - start_stable_time) >= 5.0:
                                stabilized = True
                                self.log_message("Pressure stabilized for 5 seconds within tolerance. Beginning 10s measurement period.")
                                
                        time.sleep(0.1)
                
                if not self.is_calibrating:
                    continue # Skip to the end of the outer loop if stopped.
                
                # --- Measurement Phase (10-second timer) ---
                start_measurement_time = time.time()
                readings_for_average = []
                while (time.time() - start_measurement_time) < 10:
                    if not self.is_calibrating:
                        break
                        
                    current_pressure = self.baratron.get_pressure()
                    current_valve_position = self.baratron.get_valve_position()
                    
                    if current_pressure is not None and current_valve_position is not None:
                        self.live_time_history.append(time.time() - self.start_global_time)
                        self.live_pressure_history.append(current_pressure)
                        self.live_valve_position_history.append(current_valve_position)

                        if abs(current_pressure - sp) <= measurement_tolerance:
                            readings_for_average.append(current_pressure)
                    
                    time.sleep(0.1)

                if not self.is_calibrating:
                    continue
                
                if readings_for_average:
                    mean_pressure = sum(readings_for_average) / len(readings_for_average)
                    
                    self.log_message(f"  Setpoint: {sp} Torr | Measured (Mean): {mean_pressure:.3f} Torr")
                    self.setpoint_data.append(sp)
                    self.measured_data.append(mean_pressure)
                    
                    self.live_cal_plot.set_data(self.setpoint_data, self.measured_data)
                    self.canvas.draw_idle()
                else:
                    self.log_message(f"Could not get any valid pressure readings for setpoint {sp}.")
            
            if self.is_calibrating: # Only save data if the process completed successfully
                end_time = time.time()
                total_time = end_time - self.start_global_time
                
                self.log_message("\nCalibration complete. Saving data to 'baratron_calibration.csv'...")
                df = pd.DataFrame(zip(self.setpoint_data, self.measured_data), columns=["Setpoint_Torr", "Measured_Torr"])
                df.to_csv("baratron_calibration.csv", index=False)
                self.log_message("Data saved.")
                
                # --- Calculate and log deviation statistics ---
                deviations = [abs(measured - setpoint) for setpoint, measured in zip(self.setpoint_data, self.measured_data)]
                if deviations:
                    avg_deviation = sum(deviations) / len(deviations)
                    max_deviation = max(deviations)
                    
                    self.log_message("\n--- Calibration Summary ---")
                    self.log_message(f"Total Calibration Time: {total_time:.2f} seconds")
                    self.log_message(f"Average Absolute Deviation: {avg_deviation:.4f} Torr")
                    self.log_message(f"Maximum Deviation: {max_deviation:.4f} Torr")
                    
                else:
                    self.log_message("\n--- Calibration Summary ---")
                    self.log_message("No data points were successfully measured.")
            
        except Exception as e:
            self.log_message(f"An error occurred during the calibration process: {e}")
            
        finally:
            self.is_calibrating = False
            # Ensure the valve is closed upon completion or error.
            if self.baratron and self.baratron.is_connected:
                self.log_message("Closing valve and disconnecting from the instrument.")
                self.baratron.close_valve()
                self.baratron.close()
            self.after(100, lambda: self.start_button.config(state=tk.NORMAL))
            self.after(100, lambda: self.e_stop_button.config(state=tk.DISABLED))
            self.after(100, lambda: self.fs_dropdown.config(state=tk.NORMAL))


    def send_command(self, event=None):
        """
        Sends a command from the terminal input field to the instrument.
        This is called when the user presses Enter.
        """
        command = self.command_entry.get().strip()
        self.command_entry.delete(0, tk.END)
        
        if not command:
            return
            
        self.log_message(f"> {command}")
        
        if self.baratron and self.baratron.is_connected:
            try:
                # Decide if it's a query or a write based on the command ending with '?'.
                if command.endswith('?'):
                    response = self.baratron.query_command(command)
                    if response:
                        self.log_message(f"Response: {response}")
                    else:
                        self.log_message("No response received.")
                else:
                    self.baratron.write_command(command)
                    self.log_message("Command sent.")
            except pyvisa.VisaIOError as e:
                self.log_message(f"Connection error: {e}")
            except Exception as e:
                self.log_message(f"Error: {e}")
        else:
            self.log_message("Instrument not connected. Cannot send command.")

    def on_closing(self):
        """
        Handles the graceful shutdown of the application.
        """
        self.log_message("Closing application. Disconnecting from instrument...")
        self.is_calibrating = False # Ensure the thread stops
        if self.baratron and self.baratron.is_connected:
            try:
                self.baratron.close_valve()
                time.sleep(1)
                self.baratron.close()
            except Exception as e:
                self.log_message(f"Error during shutdown: {e}")
        self.destroy()

# =================================================================================
# Main Script Execution
# This is the main part of the script that runs the GUI application.
# =================================================================================

if __name__ == "__main__":
    # !!! IMPORTANT !!!
    # This variable must be changed to match the correct address of your instrument.
    PRESSURE_CONTROLLER_ADDRESS = 'ASRL/dev/ttyUSB0::INSTR'
    
    # Launch the GUI first. All user input and control will happen here.
    app = BaratronGUI(PRESSURE_CONTROLLER_ADDRESS)
    app.mainloop()

