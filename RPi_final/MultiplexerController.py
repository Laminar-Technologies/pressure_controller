# MultiplexerController.py
# (Runs directly on the Raspberry Pi)
import RPi.GPIO as GPIO
import time

class MultiplexerController:
    """
    Manages GPIO control for both the MKS 274B Multiplexer (channel selection)
    and the MKS 270B Signal Conditioner (range selection).
    """
    def __init__(self, channel_pins, range_pins, log_queue):
        self.log_queue = log_queue
        # Pin maps provided by your setup
        self.CHANNEL_PINS = channel_pins  # e.g., {1: 10, 2: 26, 3: 16}
        self.RANGE_PINS = range_pins      # e.g., {'x0.1': 21, 'x0.01': 24}
        
        self.all_pins = list(self.CHANNEL_PINS.values()) + list(self.RANGE_PINS.values())
        self.current_channel = None

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for pin in self.all_pins:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
            self.log_queue.put("✅ Multiplexer and Range GPIO pins initialized.")
        except Exception as e:
            self.log_queue.put(f"⚠️ GPIO ERROR: Could not initialize pins. Is RPi.GPIO library installed? Error: {e}")


    def select_channel(self, channel):
        """Selects a multiplexer channel by pulling its pin LOW."""
        if channel not in self.CHANNEL_PINS:
            self.log_queue.put(f"ERROR: Invalid channel '{channel}' selected.")
            return

        if self.current_channel == channel:
            return # No change needed

        for pin_channel, pin_num in self.CHANNEL_PINS.items():
            GPIO.output(pin_num, GPIO.LOW if pin_channel == channel else GPIO.HIGH)
        
        self.current_channel = channel
        self.log_queue.put(f">> Switched to Standard Channel {channel}.")

    def set_range(self, range_multiplier):
        """Sets the 270B range based on the multiplier (1, 0.1, or 0.01)."""
        pin_x01 = self.RANGE_PINS.get('x0.1')
        pin_x001 = self.RANGE_PINS.get('x0.01')

        if range_multiplier == 1:
            GPIO.output(pin_x01, GPIO.HIGH)
            GPIO.output(pin_x001, GPIO.HIGH)
            self.log_queue.put(">> Set 270B range to x1.0.")
        elif range_multiplier == 0.1:
            GPIO.output(pin_x01, GPIO.LOW)
            GPIO.output(pin_x001, GPIO.HIGH)
            self.log_queue.put(">> Set 270B range to x0.1.")
        elif range_multiplier == 0.01:
            GPIO.output(pin_x01, GPIO.HIGH)
            GPIO.output(pin_x001, GPIO.LOW)
            self.log_queue.put(">> Set 270B range to x0.01.")
        else:
            self.log_queue.put(f"ERROR: Invalid range multiplier '{range_multiplier}'.")

    def cleanup(self):
        """Resets all GPIO pins to a safe state."""
        GPIO.cleanup(self.all_pins)
        self.log_queue.put("Multiplexer GPIO cleanup complete.")