# ==============================================================================
# Script Name: Baratron Pressure Controller Calibration Script
# Author: Gemini
# Date: August 12, 2025
# Description: This script automates the calibration of a Baratron pressure
#              transducer using an MKS 651 Series pressure controller. It
#              communicates with the controller via a serial (VISA) connection,
#              sets a series of pressure setpoints, waits for the pressure to
#              stabilize at each point, and logs the data to a CSV file.
#
# Prerequisites:
#   - Python 3
#   - pyvisa library (pip install pyvisa pyvisa-py)
#   - pandas library (pip install pandas)
#   - MKS 651C Pressure Controller connected via a serial-to-USB cable
#   - Correct VISA address for the serial port (e.g., 'ASRL/dev/ttyUSB0::INSTR')
# ==============================================================================

# --- Import necessary libraries ---
# 'pyvisa' is the library that lets our computer talk to lab instruments.
import pyvisa
# 'time' is a library for controlling time, like making the script pause.
import time
# 'pandas' is used for creating and saving data in a structured table format, like a spreadsheet.
import pandas as pd
# 're' is the regular expression library, used for finding specific text patterns.
import re

# =================================================================================
# BaratronController Class
# This is a blueprint for an object that can manage our MKS instruments.
# It holds all the functions (methods) needed to talk to the controller.
# =================================================================================

class BaratronController:
    """
    This class handles all communication with the MKS 651 Pressure Controller.
    Think of it as the brain that knows how to send and receive commands.
    """
    def __init__(self, pressure_visa_address, full_scale_pressure=1000.0):
        """
        This is the constructor, which runs when we create a new 'BaratronController' object.
        It sets up the connection to the physical device.

        Args:
            pressure_visa_address (str): The unique address for the instrument on the computer.
                                         For a USB-serial port on Linux, it often looks like
                                         'ASRL/dev/ttyUSB0::INSTR'.
            full_scale_pressure (float): The maximum pressure the transducer can measure, in Torr.
                                         This is important for calculations.
        """
        try:
            # Create a resource manager to find and open the instrument.
            self.rm = pyvisa.ResourceManager()
            
            # Open the connection to the MKS 651 Pressure Controller using its address.
            self.pressure_controller = self.rm.open_resource(pressure_visa_address)
            
            # Set the communication settings for the connection. These must match
            # the settings on the MKS 651 controller itself.
            self.pressure_controller.timeout = 5000  # Give it up to 5 seconds to respond.
            self.pressure_controller.baud_rate = 9600 # The speed of data transfer.
            self.pressure_controller.data_bits = 8    # How many bits per data packet.
            self.pressure_controller.parity = pyvisa.constants.Parity.none
            self.pressure_controller.stop_bits = pyvisa.constants.StopBits.one
            
            # These lines tell the script what character signals the end of a command
            # or a response from the controller. The MKS manual specifies '\r' (carriage return).
            self.pressure_controller.read_termination = '\r'
            self.pressure_controller.write_termination = '\r'
            
            # Save the full-scale pressure value for later calculations.
            self.full_scale_pressure = full_scale_pressure
            
            print("Successfully connected to the MKS 651 Pressure Controller.")
            
        except pyvisa.VisaIOError as e:
            # If something goes wrong, this block of code will catch the error and
            # print a helpful message, then stop the script.
            print(f"Error connecting to the instrument: {e}")
            print("Please check the VISA address and physical connection.")
            try:
                # This tries to list all available instruments to help with debugging.
                print("\nAvailable VISA resources:")
                for resource in self.rm.list_resources():
                    print(f"  - {resource}")
            except Exception as list_e:
                print(f"Could not list available resources: {list_e}")
            raise # Re-raises the error to stop the program.

    def write_command(self, command):
        """
        Sends a command to the instrument without waiting for a response.
        This is used for commands like "set this pressure" or "open the valve."
        """
        try:
            self.pressure_controller.write(command)
            print(f"Sent command: {command}")
        except pyvisa.VisaIOError as e:
            print(f"Error writing command '{command}': {e}")
            raise

    def query_command(self, command):
        """
        Sends a command and waits for a response from the instrument.
        This is used for commands like "what is the current pressure?"
        """
        try:
            response = self.pressure_controller.query(command)
            return response
        except pyvisa.VisaIOError as e:
            print(f"Error querying command '{command}': {e}")
            return None

    def set_pressure(self, pressure):
        """
        Tells the MKS controller to achieve a specific target pressure.
        
        Args:
            pressure (float): The desired pressure in Torr.
        """
        # The MKS 651C controller expects the pressure as a percentage of its full scale.
        # We must convert our target pressure (e.g., 50 Torr) into this percentage.
        setpoint_percentage = (pressure / self.full_scale_pressure) * 100
        
        # The command format is 'S1 [percentage]'. The :.2f formats the number
        # to two decimal places.
        setpoint_command = f"S1 {setpoint_percentage:.2f}"
        self.write_command(setpoint_command)

        # The 'D1' command activates the setpoint, telling the controller to
        # start trying to reach that pressure.
        self.write_command("D1")
        
        # Now we call the function to wait until the pressure is stable.
        self.wait_for_pressure_stabilization(pressure)

    def get_pressure(self):
        """
        Asks the MKS controller for the current pressure reading and
        converts it into the correct units (Torr).

        Returns:
            float: The current pressure in Torr, or None if the reading fails.
        """
        # The 'R5' command asks the controller for its current pressure value.
        response = self.query_command("R5")
        if response:
            try:
                # Use a regular expression to find the numeric part of the response.
                # This is a reliable way to handle any extra text the controller sends.
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    # The controller sends back a number which is a percentage of full scale.
                    # We need to convert this percentage back into a Torr value.
                    percentage_reading = float(match.group())
                    pressure = (percentage_reading / 100) * self.full_scale_pressure
                    return pressure
                else:
                    print(f"Could not find a numeric value in response: '{response}'")
                    return None
            except ValueError as e:
                print(f"Could not parse pressure value from response: '{response}'. Error: {e}")
        return None

    def wait_for_pressure_stabilization(self, target_pressure, tolerance_percentage=0.5, zero_torr_tolerance=0.5, timeout=300):
        """
        This function is crucial. It repeatedly checks the pressure until it is
        stable at the target value.

        Args:
            target_pressure (float): The pressure we are waiting for.
            tolerance_percentage (float): How close to the target pressure is "stable"?
                                          This is 0.5% of the target pressure.
            zero_torr_tolerance (float): A fixed tolerance value for the 0 Torr setpoint.
            timeout (int): The maximum number of seconds to wait before giving up.
        """
        # We need a special case for the 0 Torr setpoint because a percentage-based
        # tolerance would be 0, making stabilization impossible.
        if target_pressure == 0.0:
            tolerance_value = zero_torr_tolerance
            print(f"Special case for 0 Torr. Waiting for pressure to stabilize at {target_pressure} +/- {tolerance_value:.2f} Torr...")
        else:
            # Calculate the actual pressure range based on the percentage.
            tolerance_value = target_pressure * (tolerance_percentage / 100)
            print(f"Waiting for pressure to stabilize at {target_pressure} +/- {tolerance_value:.2f} Torr ({tolerance_percentage} %)...")
        
        # Keep track of when we started waiting.
        start_time = time.time()
        
        # To make sure the pressure is truly stable, we require 5 consecutive
        # readings within the tolerance.
        stable_readings_required = 5
        stable_readings_count = 0
        
        # This loop continues until either the pressure is stable or we hit the timeout.
        while time.time() - start_time < timeout:
            current_pressure = self.get_pressure()
            if current_pressure is not None:
                # Check if the current reading is within our acceptable range (tolerance).
                if abs(current_pressure - target_pressure) <= tolerance_value:
                    stable_readings_count += 1
                    print(f"Stable reading #{stable_readings_count} of {stable_readings_required}.")
                    # If we have enough consecutive stable readings, we can exit the loop.
                    if stable_readings_count >= stable_readings_required:
                        print(f"Pressure stabilized at {current_pressure} Torr.")
                        return True
                else:
                    # If the reading is outside the tolerance, we reset our counter.
                    stable_readings_count = 0 
            # Pause for one second before checking again.
            time.sleep(1) 
            
        # If the loop finishes because of the timeout, this message is printed.
        print(f"Timeout reached. Pressure did not stabilize within {timeout} seconds.")
        return False
        
    def open_valve(self):
        """
        This function sends the command to open the pressure control valve.
        """
        self.write_command("O")
            
    def close_valve(self):
        """
        This function sends the command to close the pressure control valve.
        """
        self.write_command("C")
            
    def close(self):
        """
        This function safely closes the connection to the instrument.
        """
        if self.pressure_controller:
            self.pressure_controller.close()
        print("Disconnected from MKS instruments.")

# =================================================================================
# Main Script Execution
# This is the main part of the script that runs the calibration routine.
# =================================================================================

# This line ensures the code inside only runs when the script is executed directly.
if __name__ == "__main__":
    # --- PROMPT USER FOR FULL SCALE VALUE ---
    # This block asks the user for the full-scale pressure of their transducer.
    try:
        full_scale_value = float(input("Please enter the full-scale Torr value of your pressure transducer (e.g., 1000): "))
    except ValueError:
        print("Invalid input. Please enter a valid number.")
        exit()
        
    # !!! IMPORTANT !!!
    # This variable must be changed to match the correct address of your instrument.
    PRESSURE_CONTROLLER_ADDRESS = 'ASRL/dev/ttyUSB0::INSTR'

    # Create a list of all the pressure setpoints we want to calibrate against.
    # This list now goes from 0% to 100% of the user-provided full-scale value.
    setpoints = [round(full_scale_value * i / 100, 2) for i in range(0, 101, 10)]
    data = [] # An empty list to store our calibration data.

    baratron = None
    try:
        # Create an instance of our controller class to start the connection,
        # passing the user-provided full-scale value.
        baratron = BaratronController(PRESSURE_CONTROLLER_ADDRESS, full_scale_pressure=full_scale_value)

        print("Enabling remote control...")
        # Send the command to switch the MKS controller into remote mode.
        baratron.write_command("RE 1")
        time.sleep(1)

        # This loop goes through each pressure setpoint in our list.
        for sp in setpoints:
            print(f"\n--- Starting calibration for {sp} Torr ---")
            
            # Tell the controller to go to the current setpoint pressure.
            baratron.set_pressure(sp)
            
            # Get the final, stable pressure reading from the instrument.
            current_pressure = baratron.get_pressure()
            
            # Check if we got a valid number back.
            if current_pressure is not None:
                print(f"Actual pressure at setpoint {sp} Torr is: {current_pressure} Torr")
                # Add the setpoint and the measured pressure to our data list.
                data.append((sp, current_pressure))
            else:
                print(f"Could not get pressure for setpoint {sp}.")
            
            # Wait for a few seconds before starting the next step.
            time.sleep(5)
            
        print("\nCalibration complete. Saving data to 'baratron_calibration.csv'...")
        # Use pandas to create a table (DataFrame) from our data.
        df = pd.DataFrame(data, columns=["Setpoint_Torr", "Measured_Torr"])
        # Save the data table to a file named 'baratron_calibration.csv'.
        df.to_csv("baratron_calibration.csv", index=False)
        print("Data saved.")
        
    except Exception as e:
        # This catches any other unexpected errors and prints a message.
        print(f"An error occurred during the calibration process: {e}")
        
    finally:
        # The code in this block always runs, whether an error occurred or not.
        # This is important for cleanup, like closing the valve and the connection.
        if baratron:
            print("Closing valve and disconnecting from the instrument.")
            baratron.close_valve()
            baratron.close()

