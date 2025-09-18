# DAQ_Controller.py
# (Runs directly on the Raspberry Pi with a local ADC)
import time
import threading
import collections
import numpy as np

# --- IMPORTANT ---
# You must install the library for your specific ADC board.
# This example uses the 'pi-ads1256' library.
# Install it with: pip install pi-ads1256
try:
    import ADS1256
    import RPi.GPIO as GPIO
except ImportError:
    print("WARNING: Could not import ADS1256 or RPi.GPIO. DAQ will not function.")
    print("Please install with 'pip install pi-ads1256'")
    ADS1256 = None 

# =================================================================================
# DAQController Class for Local ADC
# =================================================================================
class DAQController:
    """
    Handles local communication with an ADS1256 ADC board on the Raspberry Pi.
    """
    def __init__(self, log_queue):
        self.log_queue = log_queue
        self.ADC = None
        self.is_connected = False
        self._stop_event = threading.Event()
        self._polling_thread = None
        
        self.data_lock = threading.Lock()
        
        # Using 4 differential channels (A0-A1, A2-A3, A4-A5, A6-A7)
        self.voltage_histories = [collections.deque(maxlen=5) for _ in range(4)]

        if ADS1256 is None:
            self.log_queue.put("⚠️ ERROR: ADS1256 library not found. DAQ cannot start.")
            return

        try:
            self.ADC = ADS1256.ADS1256()
            self.ADC.ADS1256_init()
            self.is_connected = True
            self.log_queue.put("✅ ADS1256 DAQ Initialized.")
            
            self._polling_thread = threading.Thread(target=self._data_listener_thread, daemon=True)
            self._polling_thread.start()

        except Exception as e:
            self.log_queue.put(f"⚠️ ERROR: Failed to initialize ADS1256 DAQ: {e}")
            self.is_connected = False

    def _data_listener_thread(self):
        """Continuously polls all 4 differential channels."""
        while not self._stop_event.is_set():
            try:
                # Read all 4 differential channels
                # ADS1256.ADS1256_GetChannalValue(channel) returns a raw integer value.
                # You must convert it to voltage based on your reference voltage (VREF).
                # ADC Value * (VREF / 2^23)
                # Assuming VREF is 2.5V for the ADS1256 board.
                VREF = 2.5
                
                # Channel 0: A0-A1
                val_ch0 = self.ADC.ADS1256_GetChannalValue(0) - self.ADC.ADS1256_GetChannalValue(1)
                volt_ch0 = val_ch0 * (VREF / 0x7FFFFF)
                
                # Channel 1: A2-A3
                val_ch1 = self.ADC.ADS1256_GetChannalValue(2) - self.ADC.ADS1256_GetChannalValue(3)
                volt_ch1 = val_ch1 * (VREF / 0x7FFFFF)

                # Channel 2: A4-A5
                val_ch2 = self.ADC.ADS1256_GetChannalValue(4) - self.ADC.ADS1256_GetChannalValue(5)
                volt_ch2 = val_ch2 * (VREF / 0x7FFFFF)

                # Channel 3: A6-A7
                val_ch3 = self.ADC.ADS1256_GetChannalValue(6) - self.ADC.ADS1256_GetChannalValue(7)
                volt_ch3 = val_ch3 * (VREF / 0x7FFFFF)
                
                raw_voltages = [volt_ch0, volt_ch1, volt_ch2, volt_ch3]

                with self.data_lock:
                    for i, voltage in enumerate(raw_voltages):
                        self.voltage_histories[i].append(voltage)
                
                time.sleep(0.1) # Poll at 10Hz

            except Exception as e:
                self.log_queue.put(f"ERROR during DAQ polling: {e}")
                time.sleep(1)


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

    def close(self):
        """
        Signals the background thread to stop.
        """
        self._stop_event.set()
        if self._polling_thread is not None:
            self._polling_thread.join(timeout=1)
        
        if self.is_connected and ADS1256:
            GPIO.cleanup()
        
        self.is_connected = False
        self.log_queue.put("DAQ Controller closed.")