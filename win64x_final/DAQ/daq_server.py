# daq_server.py
import socket
import time
import ADS1256
from MultiplexerController import MultiplexerController
import config # Import the config module itself
import RPi.GPIO as GPIO # Import GPIO to control CS pin directly for shutdown

def handle_command(cmd, multiplexer):
    """Parses and acts on commands from the client."""
    cmd = cmd.strip().upper()
    print(f"Received command: {cmd}")
    try:
        cmd_type, value = cmd.split(':')
        if cmd_type == 'CH':
            multiplexer.select_channel(int(value))
        elif cmd_type == 'R':
            multiplexer.set_range(value)
        else:
            print(f"Unknown command type: {cmd_type}")
    except (ValueError, TypeError) as e:
        print(f"Invalid command format or value: {cmd}. Error: {e}")

def main():
    """
    Initializes hardware and starts a server to handle commands and stream ADC data.
    """
    ADC = None
    multiplexer = None
    
    try:
        # --- Initialize ADC ---
        # The ADS1256_init() function calls config.module_init() which sets up GPIO
        ADC = ADS1256.ADS1256()
        if ADC.ADS1256_init() != 0:
            raise RuntimeError("Failed to initialize ADC.")
        
        ADC.ADS1256_SetMode(1) # Set to Differential Mode
        ADC.ADS1256_ConfigADC(ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1'],
                              ADS1256.ADS1256_DRATE_E['ADS1256_100SPS'])
        print("ADC Initialized successfully.")

        # --- Initialize Multiplexer Controller ---
        channel_pins = {1: 10, 2: 26, 3: 16}
        range_pins = {'x0.1': 21, 'x0.01': 24}
        multiplexer = MultiplexerController(channel_pins, range_pins, None)
        print("Multiplexer Initialized successfully.")

        # --- Server Setup ---
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # This allows the address to be reused immediately after the script closes
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            s.bind(('0.0.0.0', 65432))
            s.listen()
            print(f"DAQ Server listening on 0.0.0.0:65432")

            while True:
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    conn.setblocking(False)
                    buffer = ""
                    
                    while True:
                        # 1. Check for incoming commands
                        try:
                            data = conn.recv(1024).decode('utf-8')
                            if not data: break
                            buffer += data
                            while '\n' in buffer:
                                command, buffer = buffer.split('\n', 1)
                                if command: handle_command(command, multiplexer)
                        except BlockingIOError:
                            pass # No data received, continue to send data
                        except (socket.error, BrokenPipeError):
                            print("Client disconnected.")
                            break

                        # 2. Read and send ADC data
                        adc_values = ADC.ADS1256_GetAll()
                        voltages = [val * 5.0 / 0x7fffff for val in adc_values]
                        
                        data_string = ",".join(map(str, voltages)) + "\n"
                        try:
                            conn.sendall(data_string.encode('utf-8'))
                        except (socket.error, BrokenPipeError):
                            print("Client disconnected.")
                            break
                        
                        time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nServer shutting down.")
    except Exception as e:
        print(f"An error occurred during operation: {e}")
    finally:
        # --- Robust Hardware Shutdown Sequence ---
        print("Starting hardware shutdown...")
        # 1. Ensure the ADC is not selected (CS pin is HIGH) before closing SPI
        # This is a critical step to prevent the chip from getting stuck.
        try:
            GPIO.output(config.CS_PIN, GPIO.HIGH)
            print("ADC Chip Select pin set to HIGH.")
        except Exception as e:
            print(f"Could not set CS pin: {e}")

        # 2. Call the centralized cleanup function
        config.module_exit()
        print("GPIO cleanup complete. Program end.")

if __name__ == '__main__':
    main()
