# -*- coding: utf-8 -*-
# ==============================================================================
# File:         State_Machine_Controller.py
# Author:       Terrance Holmes
# Date:         September 3, 2025
# Description:  This module contains the StateMachinePressureController class.
#               It manages the dual-valve pressure control system using a
#               hybrid event-driven and adaptive polling architecture.
# ==============================================================================

import serial
import time
import re
import threading
import collections
import statistics

# =================================================================================
# StateMachinePressureController Class
# =================================================================================
class StateMachinePressureController:
    """
    Manages a dual-valve pressure system using a state machine and adaptive logic.

    This controller handles the low-level communication with both the inlet
    and outlet valve controllers. It runs background threads to continuously
    poll for pressure and valve positions, and adaptively adjusts the outlet
    valve to maintain stability based on system behavior.
    """
    def __init__(self, inlet_port, outlet_port, full_scale_pressure, log_queue):
        """
        Initializes the pressure controller.

        Args:
            inlet_port (str): COM port for the inlet valve controller.
            outlet_port (str): COM port for the outlet valve controller.
            full_scale_pressure (float): The full-scale pressure of the system in Torr.
            log_queue (queue.Queue): A queue for sending log messages to the GUI.

        Raises:
            ConnectionError: If connection to either controller fails.
        """
        self.ser_inlet, self.ser_outlet = None, None
        self.is_connected = False
        self._stop_event = threading.Event()
        self._polling_thread = None
        self._adaptive_outlet_thread = None
        self.log_queue = log_queue

        # Threading locks to prevent race conditions when writing to serial ports.
        self.inlet_lock = threading.Lock()
        self.outlet_lock = threading.Lock()

        # An event to signal that all valve movements should be paused.
        self.hold_all_valves = threading.Event()

        try:
            # Establish connections to both valve controllers.
            self.ser_inlet = serial.Serial(port=inlet_port, baudrate=9600, timeout=1, write_timeout=1)
            self.ser_outlet = serial.Serial(port=outlet_port, baudrate=9600, timeout=1, write_timeout=1)

            # System parameters and state variables.
            self.full_scale_pressure = full_scale_pressure
            self.system_setpoint = 0.0
            self.previous_setpoint = 0.0
            self.pressure_history = collections.deque(maxlen=10)
            self.is_connected = True
            self.current_pressure, self.inlet_valve_pos, self.outlet_valve_pos = None, 0.0, 0.0
            self.inlet_pos_history = collections.deque(maxlen=5)
            self.hold_outlet_valve = False # Flag to temporarily freeze the outlet valve during logging.
            self.last_log_reason = "" # To avoid spamming logs with the same message.

            # Flags for adaptive logic states.
            self.max_slope_hold = False
            self.oscillation_cooldown = False
            self.oscillation_counter = 0
            self.inlet_high_blind_active = False
            self.inlet_high_blind_start_time = 0

        except serial.SerialException as e:
            self.close()
            raise ConnectionError(f"Failed to open controller ports: {e}")

    def _write_to_inlet(self, command):
        """Thread-safe method to write a command to the inlet controller."""
        with self.inlet_lock:
            if not self.is_connected or not self.ser_inlet: return
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser_inlet.write(full_command)
                self.ser_inlet.flush()
            except serial.SerialTimeoutException:
                self.log_queue.put("ERROR: Write timeout on Inlet Controller!")

    def _query_inlet(self, command):
        """Thread-safe method to send a command and read the response from the inlet controller."""
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
        """Thread-safe method to write a command to the outlet controller."""
        with self.outlet_lock:
            if not self.is_connected or not self.ser_outlet: return
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser_outlet.write(full_command)
                self.ser_outlet.flush()
            except serial.SerialTimeoutException:
                self.log_queue.put("ERROR: Write timeout on Outlet Controller!")

    def _query_outlet(self, command):
        """Thread-safe method to send a command and read the response from the outlet controller."""
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
        """Starts the background polling and adaptive logic threads."""
        if self._polling_thread is None:
            self._stop_event.clear()
            # Thread for continuously reading pressure and valve positions.
            self._polling_thread = threading.Thread(target=self._run_polling_loop, daemon=True)
            self._polling_thread.start()
            self.log_queue.put(">> Controller polling started.")
            # Thread for the adaptive control logic for the outlet valve.
            self._adaptive_outlet_thread = threading.Thread(target=self._run_adaptive_outlet_loop, daemon=True)
            self._adaptive_outlet_thread.start()
            self.log_queue.put(">> Adaptive outlet helper started.")

    def stop(self):
        """Stops the background threads and closes valves."""
        self._stop_event.set()
        if self._polling_thread is not None:
            self._polling_thread.join(timeout=2)
        if self._adaptive_outlet_thread is not None:
            self._adaptive_outlet_thread.join(timeout=2)
        self.close_valves()
        self._polling_thread = None
        self._adaptive_outlet_thread = None

    def _run_polling_loop(self):
        """
        Background thread function to poll pressure and valve positions.
        This loop runs continuously to keep the controller's state variables updated.
        """
        while not self._stop_event.is_set():
            pressure = self.get_pressure()
            if pressure is not None:
                self.current_pressure = pressure
                self.pressure_history.append(self.current_pressure)
            self.get_valve_positions()
            time.sleep(0.2) # Poll at 5 Hz.

    def _run_adaptive_outlet_loop(self):
        """
        Background thread for the adaptive outlet valve control logic.
        This complex logic analyzes system stability, inlet valve position, and
        pressure error to make intelligent adjustments to the outlet valve,
        improving stability and settling time.
        """
        while not self._stop_event.is_set():
            # Pause if manual hold is active.
            if self.hold_all_valves.is_set() or self.hold_outlet_valve:
                time.sleep(1.0)
                continue

            # Wait until enough data is collected.
            if len(self.pressure_history) < self.pressure_history.maxlen or self.system_setpoint <= 0 or self.current_pressure is None or self.inlet_valve_pos is None:
                time.sleep(1.0)
                continue

            try:
                # Deactivate blind mode after 10 seconds.
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

                # --- Oscillation Detection Logic ---
                if is_near_setpoint:
                    pressure_oscillation_threshold = (self.system_setpoint * 0.005) + (self.full_scale_pressure * 0.001)
                    pressure_std_dev = statistics.stdev(self.pressure_history)
                    if pressure_std_dev > pressure_oscillation_threshold:
                        self.oscillation_counter += 1
                    else:
                        self.oscillation_counter = 0
                else:
                    self.oscillation_counter = 0

                # --- Adaptive Action Logic ---
                # Correct for oscillations by slightly closing the outlet.
                if self.oscillation_counter >= 2:
                    self.oscillation_cooldown = True
                    # A large error during oscillation may require a bigger correction.
                    if error > (self.full_scale_pressure * 0.05):
                        new_outlet_pos = current_outlet_pos - 2.0
                        log_reason = f"EMERGENCY DESCENT (Err: {error:+.1f})"
                    else:
                        new_outlet_pos = current_outlet_pos - 0.2
                        log_reason = f"Pressure oscillating (StdDev: {statistics.stdev(self.pressure_history):.3f} Torr)"
                    self.oscillation_counter = 0

                # If inlet is nearly closed but pressure is high, open outlet to release pressure.
                elif inlet_valve_pos < 1.0 and error > 0.1:
                    self.oscillation_cooldown = False
                    new_outlet_pos = current_outlet_pos + 0.2
                    log_reason = f"Leak Up Detected (Inlet at {inlet_valve_pos:.1f}%)"

                else:
                    step_size = 0.5
                    is_pressure_stable = statistics.stdev(self.pressure_history) < (0.005 + (self.system_setpoint * 0.001))

                    # If pressure is stable but high, open the outlet valve slightly.
                    if is_pressure_stable and error > 0.2 and not self.inlet_high_blind_active:
                        self.oscillation_cooldown = False
                        new_outlet_pos = current_outlet_pos + step_size
                        log_reason = f"Stuck high (Err: {error:+.2f} Torr)"

                    # Logic from Version 89: Inlet valve is working too hard (too open),
                    # so close the outlet slightly to reduce the load.
                    elif inlet_valve_pos < 75.0 and not self.oscillation_cooldown and not self.inlet_high_blind_active:
                        inlet_trend = inlet_valve_pos - self.inlet_pos_history[-2] if len(self.inlet_pos_history) > 1 else 0
                        if inlet_trend < -0.1:
                            self.max_slope_hold = True
                            log_reason = "Max Slope Detected (Opening)"
                        else:
                            new_outlet_pos = current_outlet_pos - step_size
                            percent_open = 100.0 - inlet_valve_pos # Inlet is inverse
                            log_reason = f"Inlet valve overworked ({percent_open:.1f}% open)"

                    # If the inlet valve is almost closed, open the outlet to give it more control range.
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

                # --- Dynamic Clamping Logic ---
                # Clamp the outlet position to a safe range that varies with the setpoint.
                min_clamp = 22.0
                setpoint_percent = (self.system_setpoint / self.full_scale_pressure) * 100.0
                if setpoint_percent >= 90.0: max_clamp = 26.0
                elif setpoint_percent > 40.0: max_clamp = 30.0
                else: max_clamp = 35.0

                clamped_pos = max(min_clamp, min(max_clamp, new_outlet_pos))

                # Apply the change if it's significant.
                if abs(clamped_pos - current_outlet_pos) > 0.1:
                    if log_reason != self.last_log_reason:
                        self.log_queue.put(f"ADAPT -> Outlet to {clamped_pos:.1f}%. Reason: {log_reason}")
                        self.last_log_reason = log_reason
                    self._write_to_outlet(f"S1 {clamped_pos:.2f}")
                    self._write_to_outlet("D1")
                else:
                    self.last_log_reason = ""

            except (statistics.StatisticsError, AttributeError, IndexError):
                # Ignore errors if data history is not ready.
                pass

            time.sleep(3.0) # Run adaptive logic every 3 seconds.

    def set_pressure(self, pressure, predicted_outlet_pos=None):
        """
        Sets a new target pressure for the system.

        Args:
            pressure (float): The target pressure in Torr.
            predicted_outlet_pos (float, optional): A predicted outlet position
                from the learning model to speed up settling. Defaults to None.
        """
        self.hold_all_valves.clear()
        self.log_queue.put(f">> New system setpoint: {pressure:.3f} Torr")
        self.previous_setpoint = self.system_setpoint
        self.system_setpoint = pressure
        # Reset state variables for the new setpoint.
        self.pressure_history.clear()
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
            # Use the learned prediction if available.
            if predicted_outlet_pos is not None:
                self.log_queue.put(f">> Applying predicted outlet position: {predicted_outlet_pos:.2f}%.")
                self._write_to_outlet(f"S1 {predicted_outlet_pos:.2f}")
                self._write_to_outlet("D1")
                outlet_was_moved = True
            # If moving from zero, set a dynamic initial position.
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

            # Temporarily disable adaptive logic after a large move.
            if self.previous_setpoint == 0:
                self.inlet_high_blind_active = True
                self.inlet_high_blind_start_time = time.time()
                self.log_queue.put(">> Adaptive logic blinded for 10s after move from zero.")

            # Wait for the outlet valve to physically move before proceeding.
            if outlet_was_moved:
                self.log_queue.put(">> Waiting for outlet valve to move...")
                time.sleep(2.0)
                self.get_valve_positions()
                time.sleep(0.2)

            # Set the primary (inlet) controller to the new pressure setpoint.
            inlet_pressure_sp_percent = (pressure / self.full_scale_pressure) * 100.0
            self._write_to_inlet(f"S1 {inlet_pressure_sp_percent:.2f}")
            self._write_to_inlet("D1")

    def get_pressure(self):
        """Queries the inlet controller for the current system pressure."""
        response = self._query_inlet("R5") # R5 reads the process variable.
        if response:
            try:
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    # Convert percentage response to pressure in Torr.
                    return (float(match.group()) / 100) * self.full_scale_pressure
            except (ValueError, IndexError): return None
        return None

    def get_valve_positions(self):
        """Queries both controllers for their current valve positions."""
        try:
            # R6 reads the control output (valve position).
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
        """Commands both valves to close and holds them."""
        self.hold_all_valves.set()
        self.log_queue.put(">> All valves commanded to close.")
        self._write_to_inlet("C")
        self._write_to_outlet("C")
        time.sleep(0.5)

    def close(self):
        """Shuts down the controller, stops threads, and closes serial ports."""
        if self.is_connected:
            self.stop()
            if self.ser_inlet and self.ser_inlet.is_open: self.ser_inlet.close()
            if self.ser_outlet and self.ser_outlet.is_open: self.ser_outlet.close()
            self.is_connected = False
