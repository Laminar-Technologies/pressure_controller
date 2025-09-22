# final/State_Machine_Controller.py
# -*- coding: utf-8 -*-
# ==============================================================================
# File:         State_Machine_Controller.py
# Author:       Terrance Holmes
# Date:         September 11, 2025
# Description:  This module contains the StateMachinePressureController class.
#
# Version Update (Logic Tuning & Manual Override):
#   - Added a manual_override_active event to pause adaptive logic.
#   - Relaxed the outlet valve clamp for high-vacuum setpoints (>90% FS).
#   - "Inlet near closed" logic now more aggressively opens the outlet.
# Version Update (Raspberry Pi Performance Fix):
#   - Replaced readline() with read_until('\r') for robust communication with MKS controllers.
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
    """
    def __init__(self, inlet_port, outlet_port, full_scale_pressure, log_queue, e_stop_event):
        """
        Initializes the pressure controller and configures hardware FS range.
        """
        self.ser_inlet, self.ser_outlet = None, None
        self.is_connected = False
        self._stop_event = threading.Event()
        self.e_stop_event = e_stop_event
        self._polling_thread = None
        self._adaptive_outlet_thread = None
        self.log_queue = log_queue

        self.inlet_lock = threading.Lock()
        self.outlet_lock = threading.Lock()

        self.hold_all_valves = threading.Event()
        self.manual_override_active = threading.Event() # <-- NEW for manual cooldown

        try:
            self.ser_inlet = serial.Serial(port=inlet_port, baudrate=9600, timeout=1, write_timeout=1)
            self.ser_outlet = serial.Serial(port=outlet_port, baudrate=9600, timeout=1, write_timeout=1)

            fs_command_map = {
                0.1: 0, 1.0: 3, 10.0: 6, 100.0: 9, 1000.0: 12
            }
            command_code = fs_command_map.get(full_scale_pressure)
            if command_code is not None:
                fs_command = f"E{command_code}"
                self.log_queue.put(f">> Configuring controllers for {full_scale_pressure} Torr FS (CMD: {fs_command}).")
                self._write_to_inlet(fs_command)
                self._write_to_outlet(fs_command)
                time.sleep(0.5)
            else:
                self.log_queue.put(f"⚠️ WARNING: No direct hardware command for {full_scale_pressure} Torr FS. Controller displays may not match.")

            self.full_scale_pressure = full_scale_pressure
            self.system_setpoint = 0.0
            self.previous_setpoint = 0.0
            self.pressure_history = collections.deque(maxlen=10)
            self.is_connected = True
            self.current_pressure, self.inlet_valve_pos, self.outlet_valve_pos = None, 0.0, 0.0
            self.inlet_pos_history = collections.deque(maxlen=10)
            self.hold_outlet_valve = False
            self.last_log_reason = ""

            self.max_slope_hold = False
            self.oscillation_cooldown = False
            self.oscillation_counter = 0
            self.inlet_high_blind_active = False
            self.inlet_high_blind_start_time = 0
            
            self.inlet_near_closed_counter = 0
            self.inlet_oscillation_counter = 0

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
                self.ser_inlet.reset_input_buffer()
                full_command = (command + '\r').encode('ascii')
                self.ser_inlet.write(full_command)
                self.ser_inlet.flush()
                # --- MODIFICATION: Use read_until for MKS controllers ---
                response_bytes = self.ser_inlet.read_until(b'\r')
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
                self.ser_outlet.reset_input_buffer()
                full_command = (command + '\r').encode('ascii')
                self.ser_outlet.write(full_command)
                self.ser_outlet.flush()
                # --- MODIFICATION: Use read_until for MKS controllers ---
                response_bytes = self.ser_outlet.read_until(b'\r')
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
        while not self._stop_event.is_set() and not self.e_stop_event.is_set():
            if self.manual_override_active.is_set():
                time.sleep(1.0)
                continue
            
            if self.hold_all_valves.is_set() or self.hold_outlet_valve:
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

                if is_near_setpoint and not self.inlet_high_blind_active:
                    pressure_oscillation_threshold = (self.system_setpoint * 0.003) + (self.full_scale_pressure * 0.0008)
                    pressure_std_dev = statistics.stdev(self.pressure_history)
                    if pressure_std_dev > pressure_oscillation_threshold:
                        self.oscillation_counter = min(self.oscillation_counter + 1, 5)
                        self.oscillation_cooldown = True
                    else:
                        self.oscillation_counter = max(self.oscillation_counter - 1, 0)
                        if self.oscillation_counter == 0:
                            self.oscillation_cooldown = False
                    
                    if len(self.inlet_pos_history) == self.inlet_pos_history.maxlen:
                        inlet_std_dev = statistics.stdev(self.inlet_pos_history)
                        if inlet_std_dev > 2.0:
                            self.inlet_oscillation_counter = min(self.inlet_oscillation_counter + 1, 5)
                        else:
                            self.inlet_oscillation_counter = max(self.inlet_oscillation_counter - 1, 0)
                else:
                    self.oscillation_counter = 0
                    self.oscillation_cooldown = False
                    self.inlet_oscillation_counter = 0

                if inlet_valve_pos > 99.5:
                    self.inlet_near_closed_counter += 1
                else:
                    self.inlet_near_closed_counter = 0

                if self.oscillation_counter >= 2:
                    if error > (self.full_scale_pressure * 0.05):
                        new_outlet_pos = current_outlet_pos - 2.0
                        log_reason = f"EMERGENCY DESCENT (Err: {error:+.1f})"
                    else:
                        new_outlet_pos = current_outlet_pos - 0.2
                        log_reason = f"Pressure oscillating (StdDev: {statistics.stdev(self.pressure_history):.3f} Torr)"
                    self.oscillation_counter = 0
                
                elif self.inlet_oscillation_counter >= 3:
                    new_outlet_pos = current_outlet_pos - 0.2
                    log_reason = f"Inlet valve oscillating (StdDev: {statistics.stdev(self.inlet_pos_history):.2f}%)"
                    self.inlet_oscillation_counter = 0

                elif inlet_valve_pos < 1.0 and error > 0.1:
                    new_outlet_pos = current_outlet_pos + 0.2
                    log_reason = f"Leak Up Detected (Inlet at {inlet_valve_pos:.1f}%)"

                else:
                    step_size = 0.5
                    is_pressure_stable = statistics.stdev(self.pressure_history) < (0.005 + (self.system_setpoint * 0.001))

                    if is_pressure_stable and error > 0.2 and not self.inlet_high_blind_active and not self.oscillation_cooldown:
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
                    
                    elif self.inlet_near_closed_counter >= 5 and self.system_setpoint > 0 and self.previous_setpoint != 0:
                        # --- MODIFIED: More aggressive action when inlet is closed ---
                        if error > (self.full_scale_pressure * 0.01):
                            new_outlet_pos = current_outlet_pos + (step_size * 2) # Double the step
                            log_reason = f"Inlet closed, pressure high. Forcing outlet open."
                        else:
                             new_outlet_pos = current_outlet_pos + step_size
                             percent_open = 100.0 - inlet_valve_pos
                             log_reason = f"Inlet valve near closed ({percent_open:.1f}% open)"
                        self.inlet_near_closed_counter = 0

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

                setpoint_percent = (self.system_setpoint / self.full_scale_pressure) * 100.0 if self.full_scale_pressure > 0 else 0

                if setpoint_percent <= 10.0:
                    min_clamp = 5.0
                    max_clamp = 85.0
                elif setpoint_percent <= 40.0:
                    min_clamp = 15.0
                    max_clamp = 50.0
                elif setpoint_percent < 90.0:
                    min_clamp = 22.0
                    max_clamp = 35.0
                else: # --- MODIFIED: Relaxed clamp for high vacuum ---
                    min_clamp = 22.0
                    max_clamp = 40.0 # Was 26.0

                clamped_pos = max(min_clamp, min(max_clamp, new_outlet_pos))

                if abs(clamped_pos - current_outlet_pos) > 0.1:
                    if log_reason != self.last_log_reason:
                        self.log_queue.put(f"ADAPT -> Outlet to {clamped_pos:.1f}%. Reason: {log_reason} [Clamp: {min_clamp}-{max_clamp}%]")
                        self.last_log_reason = log_reason
                    self._write_to_outlet(f"S1 {clamped_pos:.2f}")
                    self._write_to_outlet("D1")
                else:
                    self.last_log_reason = ""

            except (statistics.StatisticsError, AttributeError, IndexError):
                pass

            time.sleep(3.0)

    def set_pressure(self, pressure, predicted_outlet_pos=None):
        if self.e_stop_event.is_set(): return
        
        self.hold_all_valves.clear()
        self.log_queue.put(f">> New system setpoint: {pressure:.3f} Torr")
        self.previous_setpoint = self.system_setpoint
        self.system_setpoint = pressure
        self.pressure_history.clear()
        self.last_log_reason = ""
        self.max_slope_hold = False
        self.oscillation_cooldown = False

        if pressure == 0:
            self._write_to_inlet("C")
            self.log_queue.put(">> PUMP TO ZERO MODE: Waiting for inlet valve to close...")
            
            timeout = time.time() + 15
            inlet_closed = False
            while time.time() < timeout:
                if self.inlet_valve_pos is not None and self.inlet_valve_pos > 99.9:
                    self.log_queue.put("   Inlet valve confirmed closed.")
                    inlet_closed = True
                    break
                time.sleep(0.5)

            if not inlet_closed:
                self.log_queue.put("!! TIMEOUT waiting for inlet valve to close. Aborting pump-down.")
                return

            current_p = self.get_pressure()
            if current_p is not None and current_p > (self.full_scale_pressure * 0.75):
                self.log_queue.put(f"!! High pressure ({current_p:.2f} Torr) detected. Ramping outlet valve.")
                
                for i in range(10):
                    if self.e_stop_event.is_set():
                        self.log_queue.put("E-STOP triggered. Aborting ramp.")
                        break
                    pos = (i + 1) * 2.0
                    self._write_to_outlet(f"S1 {pos:.2f}")
                    self._write_to_outlet("D1")
                    self.log_queue.put(f"   Ramping outlet... {pos:.1f}%")
                    time.sleep(1.0)
                
                if not self.e_stop_event.is_set():
                    self.log_queue.put("Ramp complete. Holding at 20% for 5 seconds.")
                    
                    for _ in range(5):
                        if self.e_stop_event.is_set():
                            self.log_queue.put("E-STOP triggered. Aborting hold.")
                            break
                        time.sleep(1.0)

                    if not self.e_stop_event.is_set():
                        self.log_queue.put("First hold complete. Ramping to 25%...")
                        for i in range(10):
                            if self.e_stop_event.is_set():
                                self.log_queue.put("E-STOP triggered. Aborting second ramp.")
                                break
                            pos = 20.0 + (i + 1) * 0.5
                            self._write_to_outlet(f"S1 {pos:.2f}")
                            self._write_to_outlet("D1")
                            self.log_queue.put(f"   Ramping outlet... {pos:.1f}%")
                            time.sleep(1.0)

                    if not self.e_stop_event.is_set():
                        self.log_queue.put("Second ramp complete. Holding for 1 second.")
                        time.sleep(1.0)
                        self.log_queue.put("Hold complete. Opening outlet fully.")
            else:
                self.log_queue.put(">> PUMP TO ZERO MODE: Inlet closed, Outlet fully open.")
            
            if not self.e_stop_event.is_set():
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
                elif setpoint_percent > 10.0: initial_outlet_pos = 40.0
                else: initial_outlet_pos = 70.0
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
                if self.e_stop_event.is_set(): return
                self.log_queue.put(">> Waiting for outlet valve to move...")
                time.sleep(3.0)
                if self.e_stop_event.is_set(): return
                self.get_valve_positions()
                time.sleep(0.2)

            if not self.e_stop_event.is_set():
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
