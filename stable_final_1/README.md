Multi-Device Adaptive Calibration System

1. Overview

Welcome to the Multi-Device Adaptive Calibration System. This software is designed to automate the complex and time-consuming process of calibrating multiple pressure-sensing devices (transducers, gauges, etc.) against a trusted standard.

It orchestrates the control of a vacuum system's inlet and outlet valves to achieve precise pressure setpoints, reads data from up to four Devices Under Test (DUTs) simultaneously, and generates detailed reports to guide final tuning adjustments. Its core feature is an adaptive learning algorithm that remembers optimal valve positions, significantly speeding up calibration runs over time.

This tool is intended for lab technicians, engineers, and anyone involved in the verification and calibration of pressure measurement instruments.

2. Features

Automated Calibration: Executes a pre-defined or custom sequence of pressure points, automatically waiting for the system to stabilize before logging data.
Manual Control Mode: Provides a dedicated interface for manually setting pressure points, allowing for fine-tuning, troubleshooting, and spot-checks.
Live Data Visualization: Displays real-time plots of the standard pressure and all DUT readings, providing immediate insight into system performance.
Adaptive Learning: Intelligently learns and predicts the ideal outlet valve position for any given setpoint, drastically reducing the time it takes to stabilize pressure. This learned data persists between sessions.
Post-Run Analysis: After a calibration run, the software automatically analyzes the collected data for linearity, zero offset, and span errors, providing clear tuning suggestions.
Comprehensive Logging: Generates a detailed CSV file with all raw data and a debug plot showing the pressure and valve positions over the entire run.

3. Hardware Requirements

Before running the software, ensure the following hardware is properly connected to your computer:
Inlet Pressure Controller: An inverse-acting pressure controller (e.g., MKS) that controls the gas inlet. Connected via USB/Serial.
Outlet Pressure Controller: A direct-acting pressure controller that controls the connection to the vacuum pump. Connected via USB/Serial.
Multi-Channel DAQ: A Raspberry Pi Pico (RP2040) based Data Acquisition system for reading analog voltage outputs from the DUTs.
Computer: A Windows/Mac/Linux PC with Python 3 installed.
Cabling: Appropriate USB-to-Serial cables for all connected controllers.

4. Software Installation

To get the software running, follow these steps.

Step 1: Prerequisites

Ensure you have Python 3.6 or newer installed. You can download it from the official Python website. During installation, make sure to check the box that says "Add Python to PATH".

Step 2: Download the Software Files

Place the following four Python files and the requirements.txt file into a single folder on your computer:
Main.py
DAQ_Controller.py
State_Machine_Controller.py
Manual_Cal.py
requirements.txt

Step 3: Install Required Libraries

Open your computer's command prompt or terminal.
Navigate to the folder where you saved the files. For example: cd C:\Users\YourUser\Documents\CalibrationSoftware
Run the following command to install all necessary libraries automatically:
pip install -r requirements.txt


This will install pyserial, pandas, numpy, and matplotlib.

5. Operating Instructions

Step 1: Configure the Hardware

Connect all controllers (Inlet, Outlet, DAQ) to your computer's USB ports.
Power on the controllers and the devices under test.
Allow the vacuum system to pump down to its base pressure.

Step 2: Launch and Configure the Software

Run the main application by executing the Main.py file from your terminal:
python Main.py


The main window will appear. In the Configuration section, use the dropdown menus to select the correct COM Port for the Inlet Controller, Outlet Controller, and the DAQ.
Set the System FS (Torr) to match the full-scale pressure range of your standard reference gauge.
In the Devices Under Test (DUTs) section:
Check the Enable box for each device you want to calibrate.
Set the FS (Torr) for each enabled DUT to match its specific full-scale range.
!(https://www.google.com/search?q=https://i.imgur.com/your_image_link.png)

Step 3: Connect to Instruments

Once the configuration is set, click the Connect button.
The terminal at the bottom of the window will show connection status messages.
If successful, the live pressure and valve status indicators will activate, and the "Start Auto Cal" and "Manual Cal" buttons will become available.

Step 4: Choose an Operating Mode

You now have two options: Automated Calibration or Manual Control.

Option A: Start Auto Cal (Recommended)

Click the Start Auto Cal button.
The system will begin automatically stepping through a series of pressure setpoints.
Observe:
The Terminal will log every action, including setpoint changes, stability checks, and final data points.
The Live Pressure Trace plot will show the system pressure and DUT readings over time.
The DUT Deviation plot on the right will populate with bar graphs showing the error of each DUT at each logged setpoint.
E-Stop: If you need to stop the process for any reason, click the red E-Stop button. This will safely halt the calibration, close all valves, and save any learned data.

Option B: Manual Cal

Click the Manual Cal button to enter the manual control interface.
Set Pressure: Use the preset buttons (0%, 10%, etc.) or enter a custom value in Torr and click "Set". The system will automatically try to achieve and hold this pressure.
Focus Control: Select the "Standard" view to see all DUTs plotted against the standard, or select a specific DUT to see a larger, more detailed plot for that single device.
Auto-Learning: When the system holds a stable pressure in manual mode, it automatically learns and saves the optimal valve positions. This makes future runs in Auto Cal mode much faster.
Click Exit Manual Cal to return to the main screen.

Step 5: Review the Results

Once the auto-calibration is complete, the following outputs are generated:
Tuning Suggestions Window: A window will pop up with a detailed analysis for any DUT that appears to need adjustment. It provides a diagnosis and a recommended plan of action (e.g., "ADJUST ZERO", "ADJUST SPAN").
calibration_results.csv: A CSV file is saved in the same folder as the application. This file contains a clean table of all setpoints and the corresponding averaged pressure readings from the standard and each DUT.
calibration_debug_trace.png: An image file is saved that shows a detailed plot of the pressures and valve positions over the entire duration of the run. This is useful for troubleshooting system behavior.
learned_outlet_positions.json: This file is automatically created and updated. It stores the system's learned valve positions. You do not need to edit this file.

6. Troubleshooting

Connection Error:
Verify that the correct COM ports are selected in the GUI. You can check Windows Device Manager to see which ports your devices are assigned to.
Ensure all devices are powered on and their USB cables are securely connected.
Pressure Won't Stabilize:
This is often an issue with the physical vacuum system. Check for leaks in your chamber or plumbing.
The adaptive logic is robust but may struggle with very large, unstable systems. The manual mode can be used to find a rough valve position to help the automated process.

Software Crashes or Gives an Error:
Make sure all required libraries were installed correctly using the pip install -r requirements.txt command.
