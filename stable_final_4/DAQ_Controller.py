# final/DAQ_Controller.py
# -*- coding: utf-8 -*-
# ==============================================================================
# File:         DAQ_Controller.py
# Author:       Gemini
# Date:         September 12, 2025
# Description:  This module contains the DAQController class, which now handles
#               network communication with the Raspberry Pi-based DAQ server.
#
# Version Update (Moving Average Filter):
#   - Implemented a moving average filter to smooth out high-resolution DAQ
#     readings for better stability against the standard.
#   - Replaced direct voltage reading with an average of the last 20 samples.
# ==============================================================================

import socket
import time
import threading
import collections
import numpy as np

# =================================================================================
# DAQController Class
# =================================================================================
class DAQController:
    """
    Handles network communication with the Raspberry Pi DAQ Server.
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.is_connected = False
        self._stop_event = threading.Event()
        self._polling_thread = None
        
        self.data_lock = threading.Lock()
        
        # --- NEW: Moving average components ---
        # A deque will store the last 20 readings for each channel.
        # This value can be tuned to increase or decrease smoothing.
        self.voltage_histories = [collections.deque(maxlen=5) for _ in range(4)]

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, self.port))
            self.is_connected = True
            
            self._polling_thread = threading.Thread(target=self._data_listener_thread, daemon=True)
            self._polling_thread.start()

        except (socket.timeout, socket.error) as e:
            raise ConnectionError(f"Failed to connect to DAQ at {self.host}:{self.port} - {e}")

    def _data_listener_thread(self):
        buffer = ""
        while not self._stop_event.is_set():
            try:
                data = self.sock.recv(1024).decode('utf-8')
                if not data:
                    self.is_connected = False
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    if line.strip():
                        raw_voltages = [float(v) for v in line.strip().split(',') if v]
                        if len(raw_voltages) == 4:
                            with self.data_lock:
                                # --- MODIFIED: Append new readings to the history deques ---
                                for i, voltage in enumerate(raw_voltages):
                                    self.voltage_histories[i].append(voltage)

            except (ConnectionResetError, BrokenPipeError):
                self.is_connected = False
                break
            except Exception:
                continue

    def read_voltage(self, channel):
        """
        Gets the moving average of the voltage for a specified DAQ channel.
        """
        if not self.is_connected:
            return None
        
        with self.data_lock:
            # --- MODIFIED: Return the average of the history ---
            history = self.voltage_histories[channel]
            if not history:
                return 0.0  # Return 0 if no data has been received yet
            
            # Calculate and return the mean of the collected voltages
            return np.mean(history)

    def close(self):
        """
        Signals the background thread to stop and closes the socket connection.
        """
        self._stop_event.set()
        if self._polling_thread is not None:
            self._polling_thread.join(timeout=1)
        
        if self.is_connected and self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except socket.error:
                pass
        self.is_connected = False