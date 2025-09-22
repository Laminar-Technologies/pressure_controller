# test_daq.py
import time
import ADS1256
import config

try:
    print("Initializing ADC...")
    # Initialize the ADC using its own init method
    ADC = ADS1256.ADS1256()
    if ADC.ADS1256_init() != 0:
        raise RuntimeError("Failed to initialize ADC hardware.")

    print("ADC Initialized Successfully!")

    # Take a few readings
    for i in range(5):
        raw_value = ADC.ADS1256_GetChannalValue(0) # Read from Channel 0
        voltage = (raw_value * 5.0) / 0x7fffff # Using 5.0V reference
        print(f"Reading {i+1}: Channel 0 Raw: {raw_value}, Voltage: {voltage:.4f} V")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Use the new, reliable cleanup function from config.py
    config.module_exit()
    print("GPIO cleanup complete.")
