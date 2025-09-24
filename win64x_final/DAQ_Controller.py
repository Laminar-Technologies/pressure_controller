# DAQ_Controller.py
import socket
import time
import threading
import collections
import numpy as np

try:
    import RPi.GPIO as GPIO
    from MultiplexerController import MultiplexerController
    import ADS1256
    IS_PI = True
except ImportError:
    IS_PI = False

class DAQController:
    """
    Handles network communication with the Raspberry Pi DAQ Server,
    or directly controls the DAQ if running on a Raspberry Pi.
    """
    def __init__(self, host, port, log_queue):
        self.log_queue = log_queue
        if IS_PI:
            self.ADC = ADS1256.ADS1256()
            if self.ADC.ADS1256_init() != 0:
                self.log_queue.put("ERROR: Failed to initialize ADS1256 DAQ.")
                raise RuntimeError("Failed to initialize ADC.")
            self.ADC.ADS1256_SetMode(1)
            self.ADC.ADS1256_ConfigADC(ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1'], ADS1256.ADS1256_DRATE_E['ADS1256_100SPS'])
            
            channel_pins = {1: 10, 2: 26, 3: 16}
            range_pins = {'x0.1': 21, 'x0.01': 24}
            self.multiplexer = MultiplexerController(channel_pins, range_pins, self.log_queue)
            
            self.log_queue.put("✅ ADS1256 DAQ Initialized.")
        else:
            self.host = host
            self.port = port
            self.sock = None
            self.is_connected = False
            self._stop_event = threading.Event()
            self._polling_thread = None
            self.data_lock = threading.Lock()
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
                    self.log_queue.put("DAQ WARNING: Connection to Pi lost.")
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            raw_voltages = [float(v) for v in line.strip().split(',') if v]
                            if len(raw_voltages) == 4:
                                with self.data_lock:
                                    for i, voltage in enumerate(raw_voltages):
                                        self.voltage_histories[i].append(voltage)
                        except (ValueError, IndexError):
                            self.log_queue.put(f"DAQ WARNING: Received malformed data: {line}")
            except (ConnectionResetError, BrokenPipeError):
                self.is_connected = False
                self.log_queue.put("DAQ WARNING: Connection to Pi was forcibly closed.")
                break
            except socket.error:
                self.is_connected = False
                self.log_queue.put("DAQ WARNING: Socket error. Disconnecting.")
                break

    def read_voltage(self, channel):
        if IS_PI:
            adc_values = self.ADC.ADS1256_GetAll()
            return adc_values[channel] * 5.0 / 0x7fffff
        else:
            if not self.is_connected:
                return None
            with self.data_lock:
                history = self.voltage_histories[channel]
                if not history:
                    return 0.0
                return np.mean(history)

    def select_channel(self, channel):
        if IS_PI:
            self.multiplexer.select_channel(channel)
        else:
            self._send_command(f"CH:{channel}")

    def set_range(self, range_multiplier_str):
        if IS_PI:
            self.multiplexer.set_range(range_multiplier_str)
        else:
            self._send_command(f"R:{range_multiplier_str}")

    def _send_command(self, command):
        """Sends a command string to the DAQ server."""
        if not self.is_connected:
            self.log_queue.put("DAQ ERROR: Not connected, cannot send command.")
            return
        try:
            full_command = (command + '\n').encode('utf-8')
            self.sock.sendall(full_command)
        except socket.error as e:
            self.log_queue.put(f"DAQ ERROR: Failed to send command '{command}': {e}")
            self.is_connected = False

    def close(self):
        if IS_PI:
            self.multiplexer.cleanup()
        else:
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