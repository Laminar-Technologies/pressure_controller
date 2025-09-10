# -*- coding: utf-8 -*-
# ==============================================================================
# File:         DAQ_Controller.py
# Author:       Terrance Holmes
# Date:         September 3, 2025
# Description:  This module contains the DAQController class, which handles
#               all communication with the Multi-Channel RP2040 DAQ device.
#               It provides methods for reading voltages from specified
#               channels and managing the serial connection.
# ==============================================================================

import serial
import time
import collections

# =================================================================================
# DAQController Class (for Multi-Channel RP2040)
# =================================================================================
class DAQController:
    """
    Handles communication with the Multi-Channel RP2040 DAQ.

    This class encapsulates the serial communication protocol for the DAQ,
    including connecting, reading smoothed voltage values from different
    channels, and properly closing the connection.
    """
    def __init__(self, port):
        """
        Initializes the DAQ controller and opens the serial port.

        Args:
            port (str): The COM port where the DAQ is connected (e.g., 'COM7').

        Raises:
            ConnectionError: If the specified serial port cannot be opened.
        """
        try:
            # Establish serial connection with a 2-second timeout.
            self.ser = serial.Serial(port, 9600, timeout=2)
            # A short delay to allow the device to initialize.
            time.sleep(2)
            self.is_connected = True
            # Use a deque to store the last 5 voltage readings for smoothing.
            self.voltage_history = {i: collections.deque(maxlen=12) for i in range(4)}
        except serial.SerialException as e:
            # Raise a more specific error if the connection fails.
            raise ConnectionError(f"Failed to open DAQ port {port}: {e}")

    def read_voltage(self, channel):
        """
        Reads a smoothed voltage from a specified DAQ channel.

        It sends the read command, parses the response, and returns a moving
        average of the last few readings to reduce noise.

        Args:
            channel (int): The DAQ channel to read from (0-3).

        Returns:
            float: The smoothed voltage reading, or None if the read fails.
        """
        if not self.is_connected:
            return None
        try:
            # Command format is 'R' followed by the channel number (e.g., 'R0').
            command = f'R{channel}'.encode('ascii')
            self.ser.write(command)
            self.ser.flush()  # Ensure the command is sent immediately.

            # Read the response from the DAQ.
            response = self.ser.readline().decode('ascii', errors='ignore').strip()
            raw_voltage = float(response)

            # Add the new reading to the history for smoothing.
            self.voltage_history[channel].append(raw_voltage)

            # Calculate the smoothed voltage (moving average).
            if not self.voltage_history[channel]:
                return None
            smoothed_voltage = sum(self.voltage_history[channel]) / len(self.voltage_history[channel])
            return smoothed_voltage
        except (ValueError, serial.SerialException):
            # Return None if parsing fails or a serial error occurs.
            return None

    def close(self):
        """
        Closes the serial connection to the DAQ.
        """
        if self.is_connected and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
