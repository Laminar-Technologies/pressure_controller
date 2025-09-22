# DAQ_Controller.py
import socket
import time
import threading
import collections
import numpy as np

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
            history = self.voltage_histories[channel]
            if not history:
                return 0.0
            
            return np.mean(history)

    def select_channel(self, channel):
        """Sends a command to the DAQ server to select a multiplexer channel."""
        if self.is_connected:
            try:
                command = f"CH:{channel}\n"
                self.sock.sendall(command.encode('utf-8'))
            except socket.error as e:
                print(f"Error sending channel select command: {e}")
                self.is_connected = False

    def set_range(self, multiplier):
        """Sends a command to the DAQ server to set the range."""
        if self.is_connected:
            try:
                command = f"RNG:{multiplier}\n"
                self.sock.sendall(command.encode('utf-8'))
            except socket.error as e:
                print(f"Error sending set range command: {e}")
                self.is_connected = False


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