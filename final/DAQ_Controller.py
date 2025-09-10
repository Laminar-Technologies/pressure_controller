# -*- coding: utf-8 -*-
# ==============================================================================
# File:         DAQ_Controller.py
# Author:       Gemini
# Date:         September 9, 2025
# Description:  This module contains the DAQController class, which now handles
#               network communication with the Raspberry Pi-based DAQ server.
#               It uses a thread-safe lock and a moving average filter to
#               provide clean, stable voltage readings.
#
# Version Update (Final Hardware Integration Fixes):
#   - Replaced moving average with a direct read of the latest voltage.
#   - Improved data parsing to be more robust against network interruptions.
# ==============================================================================

import socket
import time
import threading
import collections
import numpy as np # Import numpy for efficient averaging

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
        
        # --- NEW: Add a moving average filter ---
        # You can adjust this window size. A larger number gives smoother readings
        # but adds a tiny bit of lag. 5 is a good starting point.
        self.MOVING_AVERAGE_WINDOW = 5
        self.voltage_history = [collections.deque(maxlen=self.MOVING_AVERAGE_WINDOW) for _ in range(4)]
        self.latest_voltages = [0.0] * 4

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
                    
                    raw_voltages = [float(v) for v in line.split(',') if v]
                    if len(raw_voltages) == 4:
                        with self.data_lock:
                            for i in range(4):
                                # Add the new raw reading to the history deque
                                self.voltage_history[i].append(raw_voltages[i])
                                # The "latest" voltage is now the average of the history
                                self.latest_voltages[i] = np.mean(self.voltage_history[i])

            except (ConnectionResetError, BrokenPipeError):
                self.is_connected = False
                break
            except Exception:
                continue

    def read_voltage(self, channel):
        """
        Gets the latest SMOOTHED voltage for a specified DAQ channel.
        """
        if not self.is_connected:
            return None
        
        with self.data_lock:
            # This now returns the smoothed value automatically
            return self.latest_voltages[channel]

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