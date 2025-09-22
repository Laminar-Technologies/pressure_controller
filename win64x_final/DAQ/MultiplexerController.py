# DAQ/Multiplex_Controller.py
import RPi.GPIO as GPIO
import time

class MultiplexerController:
    """
    Manages GPIO control for the MKS Multiplexer and Signal Conditioner.
    Assumes GPIO.setmode() has been called by the parent script.
    """
    def __init__(self, channel_pins, range_pins, log_queue=None):
        self.log_queue = log_queue
        self.CHANNEL_PINS = channel_pins
        self.RANGE_PINS = range_pins
        
        self.all_pins = list(self.CHANNEL_PINS.values()) + list(self.RANGE_PINS.values())
        self.current_channel = None

        try:
            # Setup all pins as outputs with a safe initial state (HIGH)
            for pin in self.all_pins:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
            self._log("✅ Multiplexer and Range GPIO pins initialized.")
        except Exception as e:
            self._log(f"⚠️ GPIO ERROR: Could not initialize pins. Error: {e}")

    def _log(self, message):
        """Safely logs messages to the queue or prints to console."""
        if self.log_queue:
            self.log_queue.put(message)
        else:
            print(message) # Fallback if no queue is provided

    def select_channel(self, channel):
        """Selects a multiplexer channel by pulling its pin LOW."""
        try:
            channel = int(channel)
            if channel not in self.CHANNEL_PINS:
                self._log(f"ERROR: Invalid channel '{channel}' selected.")
                return

            if self.current_channel == channel:
                return 

            for pin_channel, pin_num in self.CHANNEL_PINS.items():
                GPIO.output(pin_num, GPIO.LOW if pin_channel == channel else GPIO.HIGH)
            
            self.current_channel = channel
            self._log(f">> Switched to Standard Channel {channel}.")
        except (ValueError, TypeError) as e:
            self._log(f"ERROR: Invalid channel value received: {channel}. Error: {e}")


    def set_range(self, range_multiplier_str):
        """Sets the 270B range based on the multiplier string ('1', '0.1', or '0.01')."""
        pin_x01 = self.RANGE_PINS.get('x0.1')
        pin_x001 = self.RANGE_PINS.get('x0.01')
        
        range_map = {
            '1': (GPIO.HIGH, GPIO.HIGH, "x1.0"),
            '0.1': (GPIO.LOW, GPIO.HIGH, "x0.1"),
            '0.01': (GPIO.HIGH, GPIO.LOW, "x0.01")
        }

        if range_multiplier_str in range_map:
            state_x01, state_x001, label = range_map[range_multiplier_str]
            GPIO.output(pin_x01, state_x01)
            GPIO.output(pin_x001, state_x001)
            self._log(f">> Set 270B range to {label}.")
        else:
            self._log(f"ERROR: Invalid range multiplier '{range_multiplier_str}'.")

    def cleanup(self):
        """Resets all GPIO pins to a safe state."""
        GPIO.cleanup(self.all_pins)
        self._log("Multiplexer GPIO cleanup complete.")
