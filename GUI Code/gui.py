import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

class PIDControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 WiFi PID Motor Controller")
        self.root.geometry("1100x680")
        
        # --- Unique Professional Engineering Theme ---
        self.bg_color = "#161925"       # Deep oxford blue/charcoal
        self.panel_color = "#232736"    # Muted navy-slate for panels
        self.text_color = "#E2E8F0"     # Crisp slate-white text
        self.accent_color = "#F5A623"   # Industrial Amber/Gold for highlights
        self.success_color = "#2ECC71"  # Clean flat green
        self.error_color = "#E74C3C"    # Clean flat red
        self.entry_bg = "#0F172A"       # Deep dark background for inputs
        
        self.root.configure(bg=self.bg_color)

        # Apply Custom Styles
        self.apply_styles()

        # --- Data Storage ---
        self.max_points = 100
        self.x_data = deque(maxlen=self.max_points)
        self.setpoint_data = deque(maxlen=self.max_points)
        self.measured_data = deque(maxlen=self.max_points)
        self.pwm_data = deque(maxlen=self.max_points)
        self.time_counter = 0

        # --- Socket Configuration ---
        self.sock = None
        self.is_reading = False
        self.esp32_port = 8080

        self.setup_ui()
        self.setup_plot()

    def apply_styles(self):
        style = ttk.Style()
        style.theme_use('clam') 

        # Frames
        style.configure('Dark.TFrame', background=self.bg_color)
        style.configure('Panel.TFrame', background=self.panel_color)

        # Labels
        style.configure('Dark.TLabel', background=self.panel_color, foreground=self.text_color, font=("Segoe UI", 10))
        style.configure('Header.TLabel', background=self.panel_color, foreground=self.accent_color, font=("Segoe UI", 11, "bold"))
        
        # Entries
        style.configure('Dark.TEntry', fieldbackground=self.entry_bg, foreground="white", insertcolor="white", borderwidth=0)

        # Buttons - Sleek Amber/Gold Button
        style.configure('Accent.TButton', background=self.accent_color, foreground="#111827", font=("Segoe UI", 10, "bold"), borderwidth=0, padding=6)
        style.map('Accent.TButton', background=[('active', '#FBBF24'), ('disabled', '#475569')])
        
        # Separator
        style.configure('Dark.TSeparator', background=self.entry_bg)

    def setup_ui(self):
        # Left Panel (Controls)
        control_frame = ttk.Frame(self.root, padding="20", style='Panel.TFrame')
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=15)

        # Title
        ttk.Label(control_frame, text="NETWORK CONNECTION", style='Header.TLabel').grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))

        # WiFi Connection setup
        ttk.Label(control_frame, text="IP Address:", style='Dark.TLabel').grid(row=1, column=0, sticky=tk.W, pady=5)
        self.ent_ip = ttk.Entry(control_frame, width=16, font=("Segoe UI", 10))
        self.ent_ip.insert(0, "192.168.8.103")
        self.ent_ip.grid(row=1, column=1, pady=5, padx=5)
        
        self.btn_connect = ttk.Button(control_frame, text="Connect via WiFi", command=self.toggle_connection, style='Accent.TButton')
        self.btn_connect.grid(row=2, column=0, columnspan=2, pady=15, sticky="ew")

        self.lbl_status = ttk.Label(control_frame, text="■ Disconnected", font=("Segoe UI", 10, "bold"), background=self.panel_color, foreground=self.error_color)
        self.lbl_status.grid(row=3, column=0, columnspan=2, pady=(0, 15))

        ttk.Separator(control_frame, orient='horizontal', style='Dark.TSeparator').grid(row=4, column=0, columnspan=2, sticky='ew', pady=15)

        # PID Inputs
        ttk.Label(control_frame, text="PID PARAMETERS", style='Header.TLabel').grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))

        ttk.Label(control_frame, text="Target RPM:", style='Dark.TLabel').grid(row=6, column=0, sticky=tk.W, pady=8)
        self.ent_rpm = ttk.Entry(control_frame, width=10, font=("Segoe UI", 11, "bold"))
        self.ent_rpm.insert(0, "100.0")
        self.ent_rpm.grid(row=6, column=1, pady=8, padx=5, sticky=tk.E)

        ttk.Label(control_frame, text="Kp (Proportional):", style='Dark.TLabel').grid(row=7, column=0, sticky=tk.W, pady=8)
        self.ent_kp = ttk.Entry(control_frame, width=10, font=("Segoe UI", 10))
        self.ent_kp.insert(0, "1.5")
        self.ent_kp.grid(row=7, column=1, pady=8, padx=5, sticky=tk.E)

        ttk.Label(control_frame, text="Ki (Integral):", style='Dark.TLabel').grid(row=8, column=0, sticky=tk.W, pady=8)
        self.ent_ki = ttk.Entry(control_frame, width=10, font=("Segoe UI", 10))
        self.ent_ki.insert(0, "0.5")
        self.ent_ki.grid(row=8, column=1, pady=8, padx=5, sticky=tk.E)

        ttk.Label(control_frame, text="Kd (Derivative):", style='Dark.TLabel').grid(row=9, column=0, sticky=tk.W, pady=8)
        self.ent_kd = ttk.Entry(control_frame, width=10, font=("Segoe UI", 10))
        self.ent_kd.insert(0, "0.05")
        self.ent_kd.grid(row=9, column=1, pady=8, padx=5, sticky=tk.E)

        self.btn_send = ttk.Button(control_frame, text="Sync Parameters", command=self.send_parameters, state=tk.DISABLED, style='Accent.TButton')
        self.btn_send.grid(row=10, column=0, columnspan=2, pady=20, sticky="ew")
        
        ttk.Separator(control_frame, orient='horizontal', style='Dark.TSeparator').grid(row=11, column=0, columnspan=2, sticky='ew', pady=10)

        # Actual RPM Display Box
        self.lbl_actual_rpm = tk.Label(control_frame, text="0.0", font=("Segoe UI", 38, "bold"), bg=self.panel_color, fg=self.accent_color)
        self.lbl_actual_rpm.grid(row=12, column=0, columnspan=2, pady=(10, 0))
        
        ttk.Label(control_frame, text="LIVE RPM", font=("Segoe UI", 10, "bold"), background=self.panel_color, foreground="#94A3B8").grid(row=13, column=0, columnspan=2, pady=0)

    def setup_plot(self):
        plot_frame = tk.Frame(self.root, bg=self.bg_color)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(0, 15), pady=15)

        # Configure Matplotlib for Custom Dark Mode
        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(7, 5), gridspec_kw={'height_ratios': [2, 1]})
        
        self.fig.patch.set_facecolor(self.bg_color)
        self.ax1.set_facecolor(self.panel_color)
        self.ax2.set_facecolor(self.panel_color)
        
        self.fig.tight_layout(pad=4.0)

        # High-end Muted Colors for Graphs
        color_setpoint = "#F87171" # Soft elegant red
        color_measured = "#60A5FA" # Clean professional blue
        color_pwm = "#34D399"      # Soft elegant green

        self.line_setpoint, = self.ax1.plot([], [], label='Target Setpoint', color=color_setpoint, linestyle='--', linewidth=1.5)
        self.line_measured, = self.ax1.plot([], [], label='Measured Speed', color=color_measured, linewidth=2)
        self.ax1.set_title("Motor Speed Telemetry", color=self.text_color, fontname="Segoe UI", fontsize=12, pad=10)
        self.ax1.set_ylabel("Revolutions Per Minute", color=self.text_color, fontname="Segoe UI")
        self.ax1.legend(loc="upper left", facecolor=self.bg_color, edgecolor=self.bg_color, prop={'family': 'Segoe UI'})
        self.ax1.grid(True, color="#334155", linestyle='-', linewidth=0.5)
        self.ax1.tick_params(colors=self.text_color)

        self.line_pwm, = self.ax2.plot([], [], label='PWM Signal', color=color_pwm, linewidth=1.5)
        self.ax2.set_title("Controller Output Effort", color=self.text_color, fontname="Segoe UI", fontsize=12, pad=10)
        self.ax2.set_ylabel("PWM (0-255)", color=self.text_color, fontname="Segoe UI")
        self.ax2.set_xlabel("Time", color=self.text_color, fontname="Segoe UI")
        self.ax2.legend(loc="upper left", facecolor=self.bg_color, edgecolor=self.bg_color, prop={'family': 'Segoe UI'})
        self.ax2.grid(True, color="#334155", linestyle='-', linewidth=0.5)
        self.ax2.tick_params(colors=self.text_color)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)

    def toggle_connection(self):
        if self.is_reading:
            self.is_reading = False
            if self.sock:
                self.sock.close()
                self.sock = None
            self.btn_connect.config(text="Connect via WiFi")
            self.btn_send.config(state=tk.DISABLED)
            self.lbl_status.config(text="■ Disconnected", foreground=self.error_color)
        else:
            ip_address = self.ent_ip.get().strip()
            if not ip_address:
                messagebox.showerror("Error", "Please enter the ESP32 IP address.")
                return
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2.0)
                self.sock.connect((ip_address, self.esp32_port))
                self.sock.settimeout(None)
                
                self.is_reading = True
                self.btn_connect.config(text="Disconnect")
                self.btn_send.config(state=tk.NORMAL)
                self.lbl_status.config(text=f"■ Connected: {ip_address}", foreground=self.success_color)
                
                threading.Thread(target=self.read_wifi_data, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not connect to {ip_address}.\n{e}")

    def send_parameters(self):
        if self.sock and self.is_reading:
            try:
                rpm = float(self.ent_rpm.get())
                kp = float(self.ent_kp.get())
                ki = float(self.ent_ki.get())
                kd = float(self.ent_kd.get())
                
                command = f"{rpm},{kp},{ki},{kd}\n"
                self.sock.sendall(command.encode('utf-8'))
            except ValueError:
                messagebox.showerror("Input Error", "Please enter valid numbers.")

    def update_rpm_display(self, rpm_val):
        self.lbl_actual_rpm.config(text=f"{rpm_val:.1f}")

    def read_wifi_data(self):
        buffer = ""
        while self.is_reading and self.sock:
            try:
                chunk = self.sock.recv(1024).decode('utf-8')
                if not chunk:
                    break 
                
                buffer += chunk
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if "Setpoint:" in line and "Measured:" in line and "PWM:" in line:
                        parts = line.split(',')
                        try:
                            sp = float(parts[0].split(':')[1])
                            meas = float(parts[1].split(':')[1])
                            pwm = float(parts[2].split(':')[1])

                            self.time_counter += 1
                            self.x_data.append(self.time_counter)
                            self.setpoint_data.append(sp)
                            self.measured_data.append(meas)
                            self.pwm_data.append(pwm)

                            self.root.after(0, self.update_rpm_display, meas)

                        except (IndexError, ValueError):
                            pass
            except Exception as e:
                break
        
        self.root.after(0, self.force_disconnect)

    def force_disconnect(self):
        if self.is_reading:
            self.toggle_connection()

    def update_plot(self, frame):
        if not self.x_data:
            return

        self.line_setpoint.set_data(self.x_data, self.setpoint_data)
        self.line_measured.set_data(self.x_data, self.measured_data)
        self.ax1.set_xlim(max(0, self.time_counter - self.max_points), self.time_counter + 5)
        
        current_sp = self.setpoint_data[-1] if self.setpoint_data else 300
        self.ax1.set_ylim(0, current_sp + 50)

        self.line_pwm.set_data(self.x_data, self.pwm_data)
        self.ax2.set_xlim(max(0, self.time_counter - self.max_points), self.time_counter + 5)
        self.ax2.set_ylim(0, 270)

        return self.line_setpoint, self.line_measured, self.line_pwm

if __name__ == "__main__":
    root = tk.Tk()
    app = PIDControlApp(root)
    root.mainloop()