# daq_server.py
import time
import socket
import ADS1256
import config

# --- ADC Setup (Runs Once) ---
V_REF = 2.5
ADC = None
try:
    ADC = ADS1256.ADS1256()
    ADC.ADS1256_init()
    ADC.ADS1256_SetMode(0)

    ADCON_REGISTER_ADDRESS = 2
    ADC.ADS1256_WriteReg(ADCON_REGISTER_ADDRESS, 0x00)
    
    print("ADS1256 Initialized: Single-Ended Mode, PGA Gain set to 1x.")
except Exception as e:
    print(f"Error initializing ADS1256: {e}")
    if ADC:
        config.module_exit()
    exit()

# --- Network Setup (Runs Once) ---
HOST = '0.0.0.0'
PORT = 65432
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen()
print(f"DAQ Server is listening on port {PORT}...")

# --- Main Server Loop ---
try:
    while True:
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            try:
                while True:
                    adc_single_ended_values = ADC.ADS1256_GetAll()
                    
                    voltages = []
                    for i in range(4):
                        positive_pin_val = adc_single_ended_values[i * 2]
                        negative_pin_val = adc_single_ended_values[i * 2 + 1]
                        
                        # Correct for inverted wiring by swapping the subtraction order.
                        raw_diff_val = negative_pin_val - positive_pin_val
                        
                        # Compensate for the 2x hardware gain.
                        scaled_volt = (raw_diff_val * V_REF * 2) / 0x7fffff
                        voltages.append(scaled_volt)

                    data_string = ",".join(f"{v:.4f}" for v in voltages) + "\n"
                    conn.sendall(data_string.encode('utf-8'))
                    time.sleep(0.2)

            except (BrokenPipeError, ConnectionResetError):
                # This block now only handles client disconnects, allowing the loop to continue.
                print(f"Client {addr} disconnected. Waiting for new connection...")
            
except KeyboardInterrupt:
    print("\nServer shutting down manually.")
except Exception as e:
    print(f"A critical error occurred: {e}")
finally:
    # This cleanup now only runs when the main loop is exited (e.g., by KeyboardInterrupt).
    s.close()
    if ADC:
        config.module_exit()
    print("Socket closed, ADC stopped, and GPIO cleaned up.")