# daq_server.py
import socket
import time
import ADS1256
import RPi.GPIO as GPIO
from MultiplexerController import MultiplexerController

# --- Network Configuration ---
HOST = '0.0.0.0'
PORT = 65432

def main():
    """
    Initializes the ADC and starts a server to stream voltage readings and receive commands.
    """
    try:
        ADC = ADS1256.ADS1256()
        if ADC.ADS1256_init() != 0:
            print("Failed to initialize ADC.")
            return

        ADC.ADS1256_SetMode(1)
        print("ADC Mode set to Differential.")
        
        ADC.ADS1256_ConfigADC(ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1'], 
                              ADS1256.ADS1256_DRATE_E['ADS1256_100SPS'])
        print("ADC Initialized successfully.")

        channel_pins = {1: 10, 2: 26, 3: 16}
        range_pins = {'x0.1': 21, 'x0.01': 24}
        multiplexer = MultiplexerController(channel_pins, range_pins, None)

    except Exception as e:
        GPIO.cleanup()
        print(f"\r\nError during ADC initialization: {e}")
        return

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"DAQ Server listening on {HOST}:{PORT}")

        while True:
            try:
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    conn.settimeout(1.0) # Set a timeout for recv

                    while True:
                        try:
                            command = conn.recv(1024).decode('utf-8')
                            if command:
                                if command.startswith("CH:"):
                                    channel = int(command.split(":")[1])
                                    multiplexer.select_channel(channel)
                                    print(f"Switched to channel {channel}")
                                elif command.startswith("RNG:"):
                                    multiplier = float(command.split(":")[1])
                                    multiplexer.set_range(multiplier)
                                    print(f"Set range to {multiplier}")
                        except socket.timeout:
                            pass # No command received, continue to send data
                        except (ValueError, IndexError):
                            print(f"Invalid command received: {command}")


                        adc_values = ADC.ADS1256_GetAll()
                        voltages = [val * 5.0 / 0x7fffff for val in adc_values]
                        
                        print(f"Sending -> DUT1(0-1): {voltages[0]:.4f}V, DUT2(2-3): {voltages[1]:.4f}V, DUT3(4-5): {voltages[2]:.4f}V, DUT4(6-7): {voltages[3]:.4f}V")

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