# daq_server.py - V6 Final with Differential Mode and Negative Voltage Fix

import socket
import time
import ADS1256
import RPi.GPIO as GPIO

# --- Network Configuration ---
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 65432        # Port to listen on

def main():
    """
    Initializes the ADC and starts a server to stream voltage readings.
    """
    try:
        ADC = ADS1256.ADS1256()
        if ADC.ADS1256_init() != 0:
            print("Failed to initialize ADC.")
            return

        # --- Set ADC to Differential Mode ---
        ADC.ADS1256_SetMode(1) 
        print("ADC Mode set to Differential.")
        
        ADC.ADS1256_ConfigADC(ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1'], 
                              ADS1256.ADS1256_DRATE_E['ADS1256_100SPS'])

        print("ADC Initialized successfully.")

    except Exception as e:
        GPIO.cleanup()
        print(f"\r\nError during ADC initialization: {e}")
        return

    # --- Server Setup ---
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"DAQ Server listening on {HOST}:{PORT}")

        while True:
            try:
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    
                    while True:
                        # This now correctly returns 4 differential values
                        adc_values = ADC.ADS1256_GetAll()
                        
                        # Convert raw ADC values to voltage
                        voltages = [val * 5.0 / 0x7fffff for val in adc_values]

                        # Print for debugging on the Pi
                        print(f"Sending -> DUT1(0-1): {voltages[0]:.4f}V, DUT2(2-3): {voltages[1]:.4f}V, DUT3(4-5): {voltages[2]:.4f}V, DUT4(6-7): {voltages[3]:.4f}V")

                        # Format and send the data
                        data_string = ",".join(map(str, voltages)) + "\n"
                        conn.sendall(data_string.encode('utf-8'))
                        
                        time.sleep(0.1) 

            except (socket.error, BrokenPipeError):
                print("Client disconnected. Waiting for a new connection...")
            except KeyboardInterrupt:
                print("\nServer shutting down.")
                break
            except Exception as e:
                print(f"An error occurred: {e}")
                break
                
    GPIO.cleanup()
    print("Program end.")

if __name__ == '__main__':
    main()
