# final/Turbo_Controller.py

import serial
import time
import threading
import re

class TurboController:
    """
    Handles all communication and safety monitoring for the Adixen ACT 200 T turbo pump controller.
    """
    def __init__(self, port, log_queue):
        """
        Initializes the controller, opens the serial port, and prepares for polling.

        Args:
            port (str): The COM port for the turbo controller.
            log_queue (queue.Queue): A queue for sending log messages to the GUI.
        """
        self.port = port
        self.log_queue = log_queue
        self.ser = None
        self.is_connected = False
        self.serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._polling_thread = None

        # --- State Variables ---
        self.rpm = 0
        self.previous_rpm = 0
        self.pump_temp = 0
        self.controller_temp = 0
        self.standby_rpm_threshold = 12000  # RPM for reduced speed standby
        self.nominal_rpm_threshold = 27000  # RPM for nominal full speed
        self.status_flags = {
            'standby': False,
            'at_speed': False,
            'is_on': False,
            'fault': False,
            'rpm_warning': False
        }

        try:
            self.ser = serial.Serial(port, 9600, timeout=1, write_timeout=1)
            self.is_connected = True
            self.log_queue.put(f"Successfully connected to Turbo Pump Controller on {port}.")
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to open Turbo Controller port {port}: {e}")

    def _query(self, command, log_it=False):
        """Thread-safe method to send a command and read a response."""
        with self.serial_lock:
            if not self.is_connected: return None
            try:
                self.ser.reset_input_buffer()
                full_command = f'#000{command}\r'.encode('ascii')
                self.ser.write(full_command)
                self.ser.flush()
                response = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # REMOVED per user request to declutter terminal
                # if log_it and not (command == "TMPON" and "Err3" in response):
                #     if command not in ["STA"]:
                #         self.log_queue.put(f"> TURBO CMD: {command} | Response: {response}")
                return response
            except serial.SerialException as e:
                self.log_queue.put(f"ERROR: Turbo serial error: {e}")
                self.is_connected = False

    def _run_polling_loop(self):
        """Background thread to continuously poll the turbo pump for status using the STA command."""
        while not self._stop_event.is_set():
            response = self._query("STA")
            if response:
                try:
                    parts = response.split(',')
                    if len(parts) >= 9:
                        status_bits = parts[1]
                        fault_bits = parts[2]
                        
                        self.status_flags['is_on'] = (status_bits[2] == '1')
                        self.status_flags['at_speed'] = (status_bits[3] == '1')
                        self.status_flags['standby'] = (status_bits[4] == '1')
                        self.status_flags['fault'] = '1' in fault_bits

                        rpm_value = int(parts[4].strip())
                        self.previous_rpm = self.rpm 
                        self.rpm = rpm_value
                        self.pump_temp = int(parts[7].strip())
                        self.controller_temp = int(parts[8].strip())
                        self._check_rpm_drop()
                except (ValueError, IndexError):
                    self.rpm = 0
                    self.pump_temp = 0
                    pass
            time.sleep(1.0)

    def _check_rpm_drop(self):
        """Monitors for a sudden, large drop in RPM."""
        rpm_drop = self.previous_rpm - self.rpm
        if self.previous_rpm > 20000 and rpm_drop > 4500:
            if not self.status_flags['rpm_warning']:
                self.log_queue.put(f"⚠️ TURBO WARNING: Sudden RPM drop of {rpm_drop} detected! Pump under stress.")
                self.status_flags['rpm_warning'] = True
        elif self.status_flags['rpm_warning'] and rpm_drop < 1000:
            self.log_queue.put("✅ TURBO INFO: RPM has recovered. Resuming normal outlet control.")
            self.status_flags['rpm_warning'] = False
            
    def start(self):
        """Starts the background polling thread."""
        if self._polling_thread is None:
            self._polling_thread = threading.Thread(target=self._run_polling_loop, daemon=True)
            self._polling_thread.start()

    def stop(self):
        """Stops the polling thread and closes the serial connection."""
        self._stop_event.set()
        if self._polling_thread is not None:
            self._polling_thread.join(timeout=1)
        if self.is_connected and self.ser:
            self.ser.close()
        self.is_connected = False
        
    def send_command(self, command, log_it=True):
        """Public method to send a command like TMPON or BRK."""
        return self._query(command, log_it)