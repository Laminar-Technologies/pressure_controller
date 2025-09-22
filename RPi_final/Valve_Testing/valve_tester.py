import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import re
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import sys # Import sys for a clean exit
import glob
import os

class ValveController:
    """Handles serial communication with a single MKS 651C valve controller."""
    def __init__(self, port, gui_app):
        self.port = port
        self.gui = gui_app
        self.ser = None
        self.is_connected = False
        self.is_inverse_mode = False
        self.serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._polling_thread = None
        self.valve_position = 0.0

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, 9600, timeout=1, write_timeout=1)
            self.is_connected = True
            
            self._send_command("T3 0")
            self.gui.log_message("Configured Setpoint C for positional control (T3 0).")

            mode_response = self._query("R32")
            if mode_response and '1' in mode_response:
                self.is_inverse_mode = True
                self.gui.log_message("Controller detected in INVERSE mode.")
            else:
                self.is_inverse_mode = False
                self.gui.log_message("Controller detected in DIRECT mode.")

            self._start_polling()
            self.gui.log_message(f"Successfully connected to controller on {self.port}.")
            return True
        except serial.SerialException as e:
            self.is_connected = False
            self.gui.log_message(f"ERROR: Could not connect to {self.port}. {e}")
            return False

    def disconnect(self):
        """Closes the serial connection in a safe order to prevent hanging."""
        if not self.is_connected:
            return
            
        self._stop_event.set()
        if self._polling_thread and self._polling_thread.is_alive():
            self._polling_thread.join(timeout=0.5)
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException as e:
                # Log silently if GUI is still available
                if self.gui:
                    self.gui.log_message(f"Error closing port: {e}")
        
        self.is_connected = False

    def _send_command(self, command):
        with self.serial_lock:
            if not self.is_connected: return
            try:
                full_command = (command + '\r').encode('ascii')
                self.ser.write(full_command)
                self.ser.flush()
            except (serial.SerialException, AttributeError):
                self.is_connected = False

    def _query(self, command):
        with self.serial_lock:
            if not self.is_connected: return None
            try:
                self.ser.reset_input_buffer()
                full_command = (command + '\r').encode('ascii')
                self.ser.write(full_command)
                self.ser.flush()
                response_bytes = self.ser.readline()
                return response_bytes.decode('ascii', errors='ignore').strip()
            except (serial.SerialException, TypeError, AttributeError):
                return None

    def _start_polling(self):
        self._stop_event.clear()
        self._polling_thread = threading.Thread(target=self._poll_position, daemon=True)
        self._polling_thread.start()

    def _poll_position(self):
        while not self._stop_event.is_set():
            response = self._query("R6")
            if response is not None:
                match = re.search(r'[+-]?\d+\.?\d*', response)
                if match:
                    self.valve_position = float(match.group(0))
            time.sleep(0.25)

    def set_position(self, percent_open):
        clamped_percent = max(0.0, min(100.0, percent_open))
        target_position = 100.0 - clamped_percent if self.is_inverse_mode else clamped_percent
        self._send_command(f"S3 {target_position:.2f}")
        self._send_command("D3")
        self.gui.log_message(f"> Setpoint C command sent: {percent_open:.2f}% open (Controller target: {target_position:.2f}%)")

    def open_valve(self):
        self.set_position(100.0)

    def close_valve(self):
        self.set_position(0.0)

class ComprehensiveValveTesterGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Comprehensive MKS Valve Tester")
        self.geometry("500x750")

        self.valve_controller = None
        self.position_var = tk.StringVar(value="--.- % Open")
        self.after_id = None
        self.is_running = True

        self.setup_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def get_serial_ports(self):
        """ Returns a list of available serial ports. """
        ports = []
        if os.name == 'posix':  # For Linux (Raspberry Pi)
            ports.extend(glob.glob('/dev/tty[A-Za-z]*'))
        elif os.name == 'nt':  # For Windows
            ports.extend([port.device for port in serial.tools.list_ports.comports()])
        return sorted(ports)

    def setup_ui(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.rowconfigure(1, weight=1)
        main_frame.columnconfigure(0, weight=1)

        top_frame = ttk.Frame(main_frame)
        top_frame.grid(row=0, column=0, sticky="ew")
        top_frame.columnconfigure(1, weight=1)

        connection_frame = ttk.LabelFrame(top_frame, text="Connection", padding=10)
        connection_frame.grid(row=0, column=0, padx=(0, 10), sticky="ns")
        ttk.Label(connection_frame, text="Serial Port:").pack(pady=2)
        self.com_port_var = tk.StringVar()
        com_ports = self.get_serial_ports()
        self.com_port_combo = ttk.Combobox(connection_frame, textvariable=self.com_port_var, values=com_ports, state="readonly", width=15)
        self.com_port_combo.pack(pady=2)
        if com_ports: self.com_port_combo.set(com_ports[0])
        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(pady=5)

        display_frame = ttk.LabelFrame(top_frame, text="Live Control & Status", padding=10)
        display_frame.grid(row=0, column=1, sticky="nsew")
        display_frame.columnconfigure(0, weight=1)
        display_frame.columnconfigure(1, weight=1)

        anim_frame = ttk.Frame(display_frame)
        anim_frame.grid(row=0, column=0, rowspan=3, sticky="ns", padx=(0, 20))
        self.fig, self.ax = plt.subplots(figsize=(2.5, 2.5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=anim_frame)
        self.canvas.get_tk_widget().pack(pady=5)
        self._draw_valve_graphic(0)
        ttk.Label(anim_frame, textvariable=self.position_var, font=("Helvetica", 24, "bold")).pack(pady=5)
        
        ttk.Label(display_frame, text="Setpoint (% Open):").grid(row=0, column=1, sticky="w", pady=(0,5))
        setpoint_frame = ttk.Frame(display_frame)
        setpoint_frame.grid(row=1, column=1, sticky="ew")
        self.setpoint_entry = ttk.Entry(setpoint_frame, width=10)
        self.setpoint_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.set_button = ttk.Button(setpoint_frame, text="Set", command=self.set_valve_position, width=5)
        self.set_button.pack(side=tk.LEFT, padx=(5,0))
        self.setpoint_entry.bind("<Return>", lambda event: self.set_valve_position())

        button_frame = ttk.Frame(display_frame)
        button_frame.grid(row=2, column=1, sticky="ew", pady=10)
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)
        self.open_button = ttk.Button(button_frame, text="Open Valve", command=self.open_valve)
        self.open_button.grid(row=0, column=0, sticky="ew", padx=(0,5))
        self.close_button = ttk.Button(button_frame, text="Close Valve", command=self.close_valve)
        self.close_button.grid(row=0, column=1, sticky="ew")
        
        term_frame = ttk.LabelFrame(main_frame, text="Terminal Log & Command Entry", padding=10)
        term_frame.grid(row=1, column=0, sticky="nsew", pady=(10,0))
        term_frame.rowconfigure(0, weight=1)
        term_frame.columnconfigure(0, weight=1)
        
        self.terminal_text = scrolledtext.ScrolledText(term_frame, height=10, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00", wrap=tk.WORD)
        self.terminal_text.grid(row=0, column=0, columnspan=2, sticky="nsew")
        self.terminal_text.config(state=tk.DISABLED)

        cmd_entry_frame = ttk.Frame(term_frame, padding=(0, 5))
        cmd_entry_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
        cmd_entry_frame.columnconfigure(1, weight=1)
        ttk.Label(cmd_entry_frame, text="CMD:").grid(row=0, column=0, padx=(0,5))
        self.command_entry = ttk.Entry(cmd_entry_frame, font=("Courier", 10))
        self.command_entry.grid(row=0, column=1, sticky="ew")
        self.command_entry.bind("<Return>", self.send_manual_command)
        self.cmd_ref_button = ttk.Button(cmd_entry_frame, text="CMD Ref", command=self.show_command_reference)
        self.cmd_ref_button.grid(row=0, column=2, padx=(5,0))
        
        self.toggle_controls(tk.DISABLED)

    def log_message(self, message):
        timestamp = time.strftime("%H:%M:%S")
        if self.is_running:
            try:
                self.terminal_text.config(state=tk.NORMAL)
                self.terminal_text.insert(tk.END, f"[{timestamp}] {message}\n")
                self.terminal_text.see(tk.END)
                self.terminal_text.config(state=tk.DISABLED)
            except tk.TclError:
                pass
    
    def toggle_connection(self):
        if self.valve_controller and self.valve_controller.is_connected:
            if self.after_id:
                self.after_cancel(self.after_id)
                self.after_id = None
            self.valve_controller.disconnect()
            self.connect_button.config(text="Connect")
            self.com_port_combo.config(state="readonly")
            self.toggle_controls(tk.DISABLED)
            self.position_var.set("--.- % Open")
            self._draw_valve_graphic(0)
        else:
            port = self.com_port_var.get()
            if not port:
                self.log_message("ERROR: No serial port selected.")
                return
            self.valve_controller = ValveController(port, self)
            if self.valve_controller.connect():
                self.connect_button.config(text="Disconnect")
                self.com_port_combo.config(state=tk.DISABLED)
                self.toggle_controls(tk.NORMAL)
                self.update_gui()

    def update_gui(self):
        if not self.is_running:
            return
            
        if self.valve_controller and self.valve_controller.is_connected:
            raw_position = self.valve_controller.valve_position
            display_position = 100.0 - raw_position if self.valve_controller.is_inverse_mode else raw_position
            self.position_var.set(f"{display_position:.1f} % Open")
            self._draw_valve_graphic(display_position)
        
        self.after_id = self.after(250, self.update_gui)

    def _draw_valve_graphic(self, percent_open):
        self.ax.clear(); self.ax.set_xlim(-1.2, 1.2); self.ax.set_ylim(-1.2, 1.2); self.ax.axis('off')
        self.ax.add_patch(patches.Circle((0, 0), 1, facecolor='#c0c0c0', edgecolor='black', linewidth=1.5))
        scaled_pos = (percent_open / 100.0) ** 0.5
        angle_deg = scaled_pos * 90.0
        ellipse_width = 2 * np.cos(np.deg2rad(angle_deg))
        butterfly = patches.Ellipse((0, 0), width=ellipse_width, height=2, facecolor='#5a5a5a', edgecolor='black')
        butterfly.set_transform(transforms.Affine2D().rotate_deg(90 - angle_deg) + self.ax.transData)
        self.ax.add_patch(butterfly)
        self.canvas.draw_idle()

    def set_valve_position(self):
        try:
            pos = float(self.setpoint_entry.get())
            if self.valve_controller: self.valve_controller.set_position(pos)
        except (ValueError):
            self.log_message("ERROR: Invalid setpoint. Please enter a number (0-100).")

    def open_valve(self):
        if self.valve_controller: self.valve_controller.open_valve()

    def close_valve(self):
        if self.valve_controller: self.valve_controller.close_valve()

    def send_manual_command(self, event=None):
        command = self.command_entry.get().strip()
        if not command: return
        self.log_message(f"> Sending command: {command}")
        if self.valve_controller:
            response = self.valve_controller._query(command)
            if response: self.log_message(f"  < Response: {response}")
            else: self.log_message("  < No response received (command sent).")
        self.command_entry.delete(0, tk.END)
            
    def toggle_controls(self, state):
        for widget in [self.setpoint_entry, self.set_button, self.open_button, self.close_button, self.command_entry]:
            widget.config(state=state)

    def show_command_reference(self):
        win = tk.Toplevel(self)
        win.title("Command and Request Reference")
        win.geometry("800x700")
        text_area = scrolledtext.ScrolledText(win, wrap=tk.WORD, font=("Courier", 10))
        text_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        text_area.tag_configure("header", font=("Courier", 10, "bold"))
        try:
            with open("Command_Reference.txt", 'r') as f:
                for line in f:
                    if line.strip().startswith("=="):
                        text_area.insert(tk.END, line, "header")
                    else:
                        text_area.insert(tk.END, line)
        except FileNotFoundError:
            text_area.insert(tk.END, "Error: Command_Reference.txt not found in the application directory.")
        text_area.config(state=tk.DISABLED)
        ttk.Button(win, text="Close", command=win.destroy).pack(pady=5)
        win.transient(self); win.grab_set()

    def on_closing(self):
        """Handles window close event cleanly."""
        self.is_running = False # Stop the update loop from re-scheduling
        
        if self.after_id:
            self.after_cancel(self.after_id)
            self.after_id = None
            
        if self.valve_controller:
            self.valve_controller.disconnect()
            
        self.destroy() # Destroy the window
        self.quit() # End the tkinter mainloop

if __name__ == "__main__":
    app = ComprehensiveValveTesterGUI()
    app.mainloop()
    # Adding sys.exit() can help ensure the process terminates on some platforms
    sys.exit()
