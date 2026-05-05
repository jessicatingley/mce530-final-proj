# MCE 530 Final Project - GUI for DC Motor Control and Stepper Motor Disc with Arduino and Simulations

# Authors: Casey Egan and Jessica Tingley
# Submission Date:

# The overwhelming majority of the code was written collaboratively, but the individual contributions are highlghted

# 
# Import necessary libraries
import tkinter as tk                         # Import tkinter libray   
from matplotlib.figure import Figure         # Import Figure class for plotting
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg # Module needed for embedded ploting        
from collections import deque                # Import deque for clearing data storage
import time

import threading
import queue
import serial

# Setting up connection to serial port for arduino data collection (make sure to change the COM port to match your system)
from pyparsing import line
import serial
ser = serial.Serial('COM6', 500000, timeout=0.1)  # Update 'COM__' to the Arduino's port

class App(tk.Tk):

    # Initialize the GUI and set up the layout with buttons and labels
    def __init__(self, window):
        # Label the window and set its size
        self.window = window
        self.window.title("MCE 530 Final")
        self.window.geometry('1150x850')
     
#### ------------------------------------------------------- ####
#### - Initialize Variables to Add Threading 
#### ------------------------------------------------------- ####
        self.running = False
        self.start_time = time.perf_counter()
        self.sample_index = 0

        self.serial_queue = queue.Queue()  # Queue to hold serial data
        self.serial_thread = None
        self.serial_thread_running = False  

# -------------------------------------
# ----  # Create Plot Controls Frame
# -------------------------------------
        plot_control_frame = tk.LabelFrame(self.window, text="Plot Controls", padx=10, pady=10, fg="purple")
        plot_control_frame.place(x=700, y=50, width=400, height=410)

        # Create the start button
        self.start_button = tk.Button(plot_control_frame, text="Start", command=self.start_plot)
        self.start_button.grid(row=0, column=0, padx=10, pady=10)
        
        # Create the stop button
        self.stop_button = tk.Button(plot_control_frame, text="Stop", command=self.stop_plot)
        self.stop_button.grid(row=0, column=1, padx=10, pady=10)
        
        # Create the clear button
        self.clear_button = tk.Button(plot_control_frame, text="Clear", command=self.clear_plot)
        self.clear_button.grid(row=0, column=2, padx=10, pady=10)

        # Create option for simulation or real connection to setup
        self.select_mode_label = tk.Label(plot_control_frame, text="Select Mode:")
        self.select_mode_label.grid(row=1, column=0, padx=10, pady=10)
        self.mode = tk.StringVar(value="simulation")     # Default to simulation mode
        self.simulation_mode_button = tk.Button(plot_control_frame, text="Simulation", command=self.select_simulation_mode)
        self.simulation_mode_button.grid(row=1, column=1, padx=10, pady=10)
        self.real_mode_button = tk.Button(plot_control_frame, text="Real Setup", command=self.select_real_lab_setup_mode)
        self.real_mode_button.grid(row=1, column=2, padx=10, pady=10)

        # Radio buttons for plot type selection
        self.plot_type_label = tk.Label(plot_control_frame, text="Plot Type:")
        self.plot_type_label.grid(row=2, column=0, padx=10, pady=10)
        self.plot_type = tk.StringVar(value="line")     # Default to line plot
        self.line_plot_type_button = tk.Radiobutton(plot_control_frame, text="Line Plot", variable=self.plot_type, value="line")
        self.line_plot_type_button.grid(row=2, column=1, padx=10, pady=10)
        self.scatter_plot_type_button = tk.Radiobutton(plot_control_frame, text="Scatter Plot", variable=self.plot_type, value="scatter")
        self.scatter_plot_type_button.grid(row=2, column=2, padx=10, pady=10)

        # Y-axis min
        self.y_min_label = tk.Label(plot_control_frame, text="Y-Min:")
        self.y_min_label.grid(row=3, column=0, padx=10, pady=10)
        self.y_min_entry = tk.Entry(plot_control_frame, width=10)
        self.y_min_entry.insert(0, "0")  # give default value so plot can be created
        self.y_min_entry.grid(row=3, column=1, padx=10, pady=10)    

        # Y-axis max
        self.y_max_label = tk.Label(plot_control_frame, text="Y-Max:")
        self.y_max_label.grid(row=4, column=0, padx=10, pady=10)
        self.y_max_entry = tk.Entry(plot_control_frame, width=10) 
        self.y_max_entry.insert(0, "6")  # give default value so plot can be created
        self.y_max_entry.grid(row=4, column=1, padx=10, pady=10)

        # Number of data points
        self.num_data_points_label = tk.Label(plot_control_frame, text="Num Data Points:")
        self.num_data_points_label.grid(row=5, column=0, padx=10, pady=10)
        self.num_data_points_entry = tk.Entry(plot_control_frame, width=10)
        self.num_data_points_entry.insert(0, "500")  # give default value so plot can be created
        self.num_data_points_entry.grid(row=5, column=1, padx=10, pady=10)

        # Create update plot button for number of data points and updating x and y limits (put in the middle of all them)
        self.update_plot_config_button = tk.Button(plot_control_frame, text="Update Plot", command=self.update_plot_config)
        self.update_plot_config_button.grid(row=4, column=2, padx=10, pady=10)

        # Create labels and entry boxes for sampling interval and send command button (frequency)
        self.sample_interval_label = tk.Label(plot_control_frame, text="Sampling Frequency (s):")
        self.sample_interval_label.grid(row=6, column=0, padx=10, pady=10)
        self.sample_interval_entry = tk.Entry(plot_control_frame, width=10)
        self.sample_interval_entry.insert(0, "0.01")  # give default value so plot can be created
        self.sample_interval_entry.grid(row=6, column=1, padx=10, pady=10)
        self.send_interval_button = tk.Button(plot_control_frame, text="Send Interval", command=self.send_interval)
        self.send_interval_button.grid(row=6, column=2, padx=10, pady=10)
        
        # Create labels and entry boxes for sampling duration and send command button
        self.sample_duration_label = tk.Label(plot_control_frame, text="Sampling Duration (s):")
        self.sample_duration_label.grid(row=7, column=0, padx=10, pady=10)
        self.sample_duration_entry = tk.Entry(plot_control_frame, width=10)
        self.sample_duration_entry.insert(0, "2")  # give default value so plot can be created
        self.sample_duration_entry.grid(row=7, column=1, padx=10, pady=10)
        self.send_duration_button = tk.Button(plot_control_frame, text="Send Duration", command=self.send_duration)
        self.send_duration_button.grid(row=7, column=2, padx=10, pady=10)
        
# -------------------------------------
# ----  # Create Feedback Control Frame
# -------------------------------------
        feedback_frame = tk.LabelFrame(self.window, text="DC Motor Feedback Controls", padx=10, pady=10, fg="green")
        feedback_frame.place(x=700, y=480, width=400, height=350)

        # Create label and entry box for step input to motor and send command button (voltage)
        self.step_input_label = tk.Label(feedback_frame, text="Desired Voltage:")
        self.step_input_label.grid(row=1, column=0, padx=10, pady=10)
        self.step_input_entry = tk.Entry(feedback_frame, width=10)
        self.step_input_entry.insert(0, "4")  # give default value so plot can be created
        self.step_input_entry.grid(row=1, column=1, padx=10, pady=10)
        self.send_step_input_button = tk.Button(feedback_frame, text="Send Step (V)", command=self.send_step_input)
        self.send_step_input_button.grid(row=1, column=2, padx=10, pady=10)

        # Create label, entry box and send button for kp
        self.kp_label = tk.Label(feedback_frame, text="Kp:")
        self.kp_label.grid(row=2, column=0, padx=10, pady=10)
        self.kp_entry = tk.Entry(feedback_frame, width=10)
        self.kp_entry.insert(0, "3")  # give default value so plot can be created
        self.kp_entry.grid(row=2, column=1, padx=10, pady=10)
        self.send_feedback_button = tk.Button(feedback_frame, text="Send Kp Gain", command=self.send_kp)
        self.send_feedback_button.grid(row=2, column=2, padx=5, pady=10)

        self.ki_label = tk.Label(feedback_frame, text="Ki:")
        self.ki_label.grid(row=3, column=0, padx=10, pady=10)
        self.ki_entry = tk.Entry(feedback_frame, width=10)
        self.ki_entry.insert(0, "5")  # give default value so plot can be created
        self.ki_entry.grid(row=3, column=1, padx=10, pady=10)
        
        # Create button to send Ki
        self.send_ki_button = tk.Button(feedback_frame, text="Send Ki Gain", command=self.send_ki)
        self.send_ki_button.grid(row=3, column=2, padx=10, pady=10)

        # Create button to remove feedback control (go to open loop and set Kp and Ki to 0)
        self.remove_feedback_button = tk.Button(feedback_frame, text="Remove Feedback (Open)", command=self.remove_feedback)
        self.remove_feedback_button.grid(row=4, column=1, padx=10, pady=10)

        # Create labels for live feedback of voltage reading and PWM sent from the arduino
        self.raw_voltage_label = tk.Label(feedback_frame, text="Live Voltage Reading: N/A", fg="green")
        self.raw_voltage_label.place(x=10, y=210)  

        self.pwm_label = tk.Label(feedback_frame, text="Live PWM Sent: N/A", fg="green")
        self.pwm_label.place(x=10, y=260)  

# -------------------------------------
# ----  # Create status bar frame for most recent command sent and error messages
# -------------------------------------
        self.status_frame = tk.LabelFrame(window, text="Status and Most Recent Commands", padx=10, pady=10)
        self.status_frame.place(x=320, y=480, width=350, height=80)

        self.status_label = tk.Label(self.status_frame, text="Status: Ready                   Errors: None", fg="green")
        self.status_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")

# -------------------------------------
# ----  # Create frame for controlling the DC motor trajectory (3 sections: each with starting V, ending V, and duration)
# -------------------------------------
        self.velocity_trajectory_frame = tk.LabelFrame(window, text="DC Motor Trajectory Controls", padx=10, pady=10, fg="dark orange")
        self.velocity_trajectory_frame.place(x=320, y=580, width=350, height=250)
        
        self.starting_velocity_label = tk.Label(self.velocity_trajectory_frame, text="Starting V:")
        self.starting_velocity_label.grid(row=0, column=1, padx=5, pady=10)
        self.ending_velocity_label = tk.Label(self.velocity_trajectory_frame, text="Ending V:")
        self.ending_velocity_label.grid(row=0, column=2, padx=5, pady=10)
        self.segment_duration_label = tk.Label(self.velocity_trajectory_frame, text="Duration (s):")
        self.segment_duration_label.grid(row=0, column=3, padx=5, pady=10)
        
        self.segment_1_label = tk.Label(self.velocity_trajectory_frame, text="Segment 1:")
        self.segment_1_label.grid(row=1, column=0, padx=5, pady=10)
        self.segment_1_start_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_1_start_entry.insert(0, "0")  
        self.segment_1_start_entry.grid(row=1, column=1, padx=5, pady=10)
        self.segment_1_end_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_1_end_entry.insert(0, "4")
        self.segment_1_end_entry.grid(row=1, column=2, padx=5, pady=10)
        self.segment_1_duration_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_1_duration_entry.insert(0, "2")
        self.segment_1_duration_entry.grid(row=1, column=3, padx=5, pady=10)

        self.segment_2_label = tk.Label(self.velocity_trajectory_frame, text="Segment 2:")
        self.segment_2_label.grid(row=2, column=0, padx=5, pady=10)
        self.segment_2_start_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_2_start_entry.insert(0, "4")
        self.segment_2_start_entry.grid(row=2, column=1, padx=5, pady=10)
        self.segment_2_end_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_2_end_entry.insert(0, "2")
        self.segment_2_end_entry.grid(row=2, column=2, padx=5, pady=10)
        self.segment_2_duration_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_2_duration_entry.insert(0, "1")
        self.segment_2_duration_entry.grid(row=2, column=3, padx=5, pady=10)

        self.segment_3_label = tk.Label(self.velocity_trajectory_frame, text="Segment 3:")
        self.segment_3_label.grid(row=3, column=0, padx=5, pady=10)
        self.segment_3_start_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_3_start_entry.insert(0, "2")
        self.segment_3_start_entry.grid(row=3, column=1, padx=5, pady=10)
        self.segment_3_end_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_3_end_entry.insert(0, "0")
        self.segment_3_end_entry.grid(row=3, column=2, padx=5, pady=10)
        self.segment_3_duration_entry = tk.Entry(self.velocity_trajectory_frame, width=10)
        self.segment_3_duration_entry.insert(0, "5")
        self.segment_3_duration_entry.grid(row=3, column=3, padx=5, pady=10)

        self.send_trajectory_button = tk.Button(self.velocity_trajectory_frame, text="Send Trajectory", command=self.send_trajectory)
        self.send_trajectory_button.place(x=200, y=180)
 

# -------------------------------------
# ----  # Create frame for controlling the stepper motor disc and displaying the net revolutions
# -------------------------------------
        self.disc_control_frame = tk.LabelFrame(window, text="Stepper Motor Disc Controls", padx=10, pady=10, fg="blue")
        self.disc_control_frame.place(x=50, y=480, width=240, height=350)

        # Radio button to set speeds with blank space vehind it to preserve grid layout
        self.blank_space = tk.Label(self.disc_control_frame, text="")
        self.blank_space.grid(row=0, column=0, padx=10, pady=20)

        self.disc_speed = tk.StringVar(value="medium")         # Default to medium spped
        self.disc_speed_label = tk.Label(self.disc_control_frame, text="Set Disc Speed:")
        self.disc_speed_label.place(x=0, y=0)
        self.slow_button = tk.Radiobutton(self.disc_control_frame, text="Slow", variable=self.disc_speed, value="slow")
        self.slow_button.place(x=10, y=25)
        self.medium_button = tk.Radiobutton(self.disc_control_frame, text="Medium", variable=self.disc_speed, value="medium")
        self.medium_button.place(x=70, y=25)
        self.fast_button = tk.Radiobutton(self.disc_control_frame, text="Fast", variable=self.disc_speed, value="fast")
        self.fast_button.place(x=150, y=25)

        # Start and stop buttons to initially home the disc or stop the motion immediately
        self.start_disc_button = tk.Button(self.disc_control_frame, text="Start/Home Disc", fg="green", command=self.start_disc_motion)
        self.start_disc_button.grid(row=1, column=0, padx=10, pady=10)

        self.stop_disc_button = tk.Button(self.disc_control_frame, text="Stop Disc", command=self.stop_disc_motion, fg="red")
        self.stop_disc_button.grid(row=1, column=1, padx=10, pady=10)


        # Buttons to send single CW or single CCW
        self.cw_button = tk.Button(self.disc_control_frame, text="Single CW", command=self.send_single_cw)
        self.cw_button.grid(row=2, column=0, padx=10, pady=10)
        self.ccw_button = tk.Button(self.disc_control_frame, text="Single CCW", command=self.send_single_ccw)
        self.ccw_button.grid(row=2, column=1, padx=10, pady=10)

        # Slider for selecting N CW commands (1-10) and send button
        self.cw_slider = tk.Scale(self.disc_control_frame, from_=2, to=10, orient=tk.HORIZONTAL)
        self.cw_slider.grid(row=3, column=0, padx=5, pady=10)
        self.cw_send_button = tk.Button(self.disc_control_frame, text="Send N CW", command=self.send_N_cw)
        self.cw_send_button.grid(row=3, column=1, padx=10, pady=10)

        # Slider for selecting M CCW commands (1-10) and send button
        self.ccw_slider = tk.Scale(self.disc_control_frame, from_=2, to=10, orient=tk.HORIZONTAL)
        self.ccw_slider.grid(row=4, column=0, padx=10, pady=10)
        self.ccw_send_button = tk.Button(self.disc_control_frame, text="Send M CCW", command=self.send_M_ccw)
        self.ccw_send_button.grid(row=4, column=1, padx=10, pady=10)

        # Label to display net revolutions, this will be updated live from the serial output of the arduino
        self.net_rev_label = tk.Label(self.disc_control_frame, text="Net Revolutions: 0", fg="blue")
        self.net_rev_label.place(x=10, y=280)

        # Create plot
        self.create_plot()

#### ------------------------------------------------------- ####
#### - Start Threading After Initialization GUI - ####
#### ------------------------------------------------------- ####
        self.serial_thread_running = True
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()
    # Function to continuosly read serial data from the Arduino
    # This will split both the motor speed (voltage reading converted from ADC) and the PWM value (control output) 
    def read_serial_data(self):
        while self.serial_thread_running:
            if ser.in_waiting > 0:
                raw_line = ser.readline().decode(errors="ignore").strip()
                
                parts = raw_line.split(",")
                
                try:
                    msg_type = parts[0]
                    
                    # DC motor sends a line with 'd' for first part
                    #  - if no trajectory: d,voltage,pwm
                    #  - if trajectory: d,voltage,pwm,target
                    if msg_type == "d":
                        voltage = float(parts[1])
                        pwm = float(parts[2])

                        target = None           # Default to no target
                        if len(parts) >=4:
                            target = float(parts[3])
                
                        self.serial_queue.put((voltage, target))       

                        self.raw_voltage_label.config(text=f"Voltage Reading: {voltage} V")  # Update the voltage reading label
                        self.pwm_label.config(text=f"PWM Sent: {pwm}")  # Update the PWM label
                    
                    # Stepper motor sends a line with 'r' at the start (for rev count)
                    elif msg_type == "r":
                        rev_count = int(float(parts[1]))
                        self.net_rev_label.config(text=f"Net Revolutions: {rev_count}")
                
                except ValueError:
                    # ignore any non-numeric junk just in case
                    pass
            else:
                time.sleep(0.001)  # Sleep briefly to prevent high CPU usage                
#### ------------------------------------------------------- ####
#### ------------------------------------------------------- ####


# -------------------------------------
# Function to create the plot based on user settings
# -------------------------------------
    def create_plot(self):
        # Data storage
        self.max_points = int(self.num_data_points_entry.get())

        # Set up data deques
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        self.target_x_data = deque(maxlen=self.max_points)
        self.target_y_data = deque(maxlen=self.max_points)

        # Create figure and axis
        graph = FigureCanvasTkAgg(Figure(figsize=(6, 4), dpi=100), master=self.window)
        graph.get_tk_widget().place(x = 50,y =50)  
        
        self.ax = graph.figure.add_subplot()
        self.ax.set_ylim(*self.get_y_limits())
        self.ax.set_title("DC Motor Response")
        self.ax.grid(True)
        self.ax.set_xlim(0, self.max_points)
        self.ax.set_ylabel("Voltage (V)")
        self.ax.set_xlabel("Sample #")

        self.canvas = graph

    # testing
    def compute_target_voltage(self, sample_index):
        interval = float(self.sample_interval_entry.get())
        t = sample_index * interval

        segments = [
            (float(self.segment_1_start_entry.get()), float(self.segment_1_end_entry.get()), float(self.segment_1_duration_entry.get())),
            (float(self.segment_2_start_entry.get()), float(self.segment_2_end_entry.get()), float(self.segment_2_duration_entry.get())),
            (float(self.segment_3_start_entry.get()), float(self.segment_3_end_entry.get()), float(self.segment_3_duration_entry.get())),
        ]

        elapsed = 0
        for start_v, end_v, duration in segments:
            if t <= elapsed + duration:
                frac = (t - elapsed) / duration
                return start_v + frac * (end_v - start_v)
            elapsed += duration
        
        return None
    

    def update_plot_config(self):
        """Ensures graph doesn't clear when num points is changed"""
        try:
            new_max = int(self.num_data_points_entry.get())
            if not (50 <= new_max <= 5000):
                self.show_error("Number of data points must be between 50 and 5000.")
                return
        except ValueError:
            self.show_error("Number of data points must be an integer.")
            return

        try:
            y_min = float(self.y_min_entry.get())
            y_max = float(self.y_max_entry.get())
            if y_min >= y_max:
                self.show_error("Y-Min must be less than Y-Max.")
                return
        except ValueError:
            self.show_error("Y-Min and Y-Max must be numbers.")
            return

        # update status bar to show plot configuration was updated
        self.status_label.config(text=f"Plot Configuration Updated", fg="Purple")
        self.clear_error()

        # Preserve existing data
        self.x_data = deque(self.x_data, maxlen=new_max)
        self.y_data = deque(self.y_data, maxlen=new_max)
        self.max_points = new_max
        self.target_x_data = deque(maxlen=self.max_points)
        self.target_y_data = deque(maxlen=self.max_points)
        self.y_min = y_min
        self.y_max = y_max

        # Update axes limits and redraw plot with new configuration
        self.ax.set_ylim(self.y_min, self.y_max)
        self.canvas.draw()


# Create empty functions to test GUI spacing and button functionality before adding in the serial communication and plotting code
    def select_simulation_mode(self):
        self.status_label.config(text="Simulation Mode Selected", fg="purple")
        self.simulation_mode_button.config(state=tk.DISABLED)
        self.real_mode_button.config(state=tk.NORMAL)
    
    def select_real_lab_setup_mode(self):
        self.status_label.config(text="Real Lab Setup Mode Selected", fg="purple")
        self.simulation_mode_button.config(state=tk.NORMAL)
        self.real_mode_button.config(state=tk.DISABLED)
    
    def start_plot(self):
        self.status_label.config(text="Plotting Started DC Motor Running", fg="green")
        self.start_button.config(state=tk.DISABLED)
        self.simulation_mode_button.config(state=tk.DISABLED)
        self.real_mode_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.running = True
        self.start_time = time.perf_counter()
        ser.write(str("s1" + "\n").encode())  # Send start command to Arduino 
                
        # Clear serial queue and buffer when restart plot to prevent old data from being plotted when starting again
        while not self.serial_queue.empty():
            try:
                self.serial_queue.get_nowait()  # Clear any remaining data in the queue
                self.serial_queue.task_done()
            except queue.Empty:
                break
        ser.reset_input_buffer()  # Clear the serial buffer to prevent old data from being read when starting again

        self.update_plot()
    
    def update_plot(self):
        if not self.running:
            return

        t = time.perf_counter() - self.start_time

        if t >= float(self.sample_duration_entry.get()):
            while not self.serial_queue.empty():
                voltage, target = self.serial_queue.get()
                self.x_data.append(self.sample_index)
                self.y_data.append(voltage)
                # testing
                target = self.compute_target_voltage(self.sample_index)
                if target is not None:
                    self.target_x_data.append(self.sample_index)
                    self.target_y_data.append(target)
                self.sample_index += 1
            self.stop_plot()
            return
        
        got_new_data = False

        while not self.serial_queue.empty():
            voltage, target = self.serial_queue.get()
            got_new_data = True

            self.x_data.append(self.sample_index)
            self.y_data.append(voltage)

            if target is not None:
                self.target_x_data.append(self.sample_index)
                self.target_y_data.append(target)

            self.sample_index += 1

        if got_new_data and len(self.x_data) > 0 and len(self.y_data) > 0:
            
            self.ax.cla()
            self.ax.set_ylim(*self.get_y_limits())

            self.ax.set_xlim(
                    max(0, self.sample_index - self.max_points),
                    max(self.sample_index, self.max_points)
                    )
            self.ax.grid(True)

            if self.plot_type.get() == "line":
                self.ax.plot(self.x_data, self.y_data, label = "Actual Read Voltage")
                if len(self.target_x_data) > 0:
                    self.ax.plot(self.target_x_data, self.target_y_data, label="Trajectory Target Voltage")
            else:
                self.ax.scatter(self.x_data, self.y_data, label="Actual Voltage")
                if len(self.target_x_data) > 0:
                    self.ax.scatter(self.target_x_data, self.target_y_data, label="Target Voltage")

            # Add legend
            self.ax.legend()

            # Keep plot labels
            self.ax.set_title("DC Motor Response")
            self.ax.set_ylabel("Voltage (V)")
            self.ax.set_xlabel("Sample #")

            self.canvas.draw()

        self.window.after(int(1000*float(self.sample_interval_entry.get())), self.update_plot)

    def stop_plot(self):
        self.status_label.config(text="Plotting Stopped", fg="red")
        self.start_button.config(state=tk.NORMAL)
        self.simulation_mode_button.config(state=tk.NORMAL)
        self.real_mode_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

        self.running = False
        ser.write(str("s0" + "\n").encode())  # Send stop command to Arduino

    def clear_plot(self):
        self.status_label.config(text="Plot Cleared", fg="purple")
        self.start_button.config(state=tk.NORMAL)
        self.running = False            
        self.x_data.clear()
        self.y_data.clear()
        self.target_x_data.clear()
        self.target_y_data.clear()
        self.sample_index = 0

        # Ensure graph gets redrawn to be fully reset
        self.ax.cla()
        self.ax.set_ylim(*self.get_y_limits())
        self.ax.set_xlim(0, self.max_points)
        self.ax.grid(True)

        # Keep the axis labels when clearing
        self.ax.set_title("DC Motor Response")
        self.ax.set_ylabel("Voltage (V)")
        self.ax.set_xlabel("Sample #")
        self.canvas.draw()

    def send_interval(self):
        self.status_label.config(text=f"Sent Interval = {self.sample_interval_entry.get()}s", fg="purple")
        ser.write(str("t" + self.sample_interval_entry.get() + "\n").encode())  # Send the sampling interval to the Arduino

    def send_duration(self):
        self.status_label.config(text=f"Sent Duration: {self.sample_duration_entry.get()}s", fg="purple")
        ser.write(str("d" + self.sample_duration_entry.get() + "\n").encode())  # Send the sampling duration to the Arduino

    
    def get_y_limits(self):
        try:
            y_min = float(self.y_min_entry.get())
            y_max = float(self.y_max_entry.get())

            # Only update min and max y values if valid 
            # Prevents graph breaking while user is entering values
            if y_min < y_max:
                self.y_min = y_min
                self.y_max = y_max
                self.clear_error()
            else:
                self.show_error("Y-Min must be less than Y-Max.")
        except ValueError:
                self.show_error("Y-Min and Y-Max must be numbers.")
        
        return self.y_min, self.y_max
        
    def send_step_input(self):
        self.status_label.config(text=f"Sent Step Input: {self.step_input_entry.get()} V", fg="green")
        ser.write(str("v" + self.step_input_entry.get() + "\n").encode())  # Send the step input voltage to the Arduino
    
    def send_kp(self):
        self.status_label.config(text=f"Sent Kp Gain: {self.kp_entry.get()}", fg="green")
        ser.write(str("p" + self.kp_entry.get() + "\n").encode()) 

    def send_ki(self):
        self.status_label.config(text=f"Sent Ki Gain: {self.ki_entry.get()}", fg="green")
        ser.write(str("i" + self.ki_entry.get() + "\n").encode())
    
    def remove_feedback(self):
        self.status_label.config(text="Removed Feedback (Open Loop)", fg="green")
        ser.write(str("o" +  "1" + "\n").encode()) 

# -------------------------------------
# ----  # Functions for Trajectory
# -------------------------------------
    #   It pulls the timing settings from the plot control frame and checks that the full trajectory can be shown with the plots duration
    #   It then creates trajectory arrays based on the starting/ending voltages and the individual durations
    #   It clears any existing plotted data before redrawing the canvas with the piecewise function
    def send_trajectory(self):
        try:
            # Get the individual segment settings
            segment_1 = [
                float(self.segment_1_start_entry.get()),
                float(self.segment_1_end_entry.get()),
                float(self.segment_1_duration_entry.get())
            ]

            segment_2 = [
                float(self.segment_2_start_entry.get()),
                float(self.segment_2_end_entry.get()),
                float(self.segment_2_duration_entry.get())
            ]

            segment_3 = [
                float(self.segment_3_start_entry.get()),
                float(self.segment_3_end_entry.get()),
                float(self.segment_3_duration_entry.get())
            ]

            # Confirm the whole trajectory can be shown
            total_segment_duration = segment_1[2] + segment_2[2] + segment_3[2]
            sample_duration = float(self.sample_duration_entry.get())
            if total_segment_duration > sample_duration:
                self.show_error("Plot Duration < Trajectory Duration. Cannot Display Full Trajectory")
                return
       
            # Combine into one list, send to arduino, and update status bar
            trajectory_values = segment_1 + segment_2 + segment_3
            ser.write(str("j" + ",".join(str(v) for v in trajectory_values) + "\n").encode())
            self.status_label.config(text="Sent Trajectory to Arduino", fg="dark orange")

        except ValueError:
            self.show_error("Invalid Value(s) in Trajectory Entries")
        

# -------------------------------------
# ----  # Functions for Disc Motion
# -------------------------------------
    def start_disc_motion(self):
        self.status_label.config(text="Sent Start Command for Disc, Homming", fg="green")
        ser.write(str("h" + "\n").encode()) # Send start command to Arduino
        self.start_disc_button.config(state=tk.DISABLED)

    def stop_disc_motion(self):
        self.status_label.config(text="Sent Stop Command for Disc", fg="red")
        ser.write(str("q" + "\n").encode())
        self.start_disc_button.config(state=tk.NORMAL)

    # Read the Radio Buttons to get speed
    def get_disc_speed(self) -> str:
        speed = self.disc_speed.get()
        if speed == "slow":
            return 'l'
        elif speed == "medium":
            return 'm'
        elif speed == "fast":
            return "f"
        return 'm'

    ##########
    # All send motion commands will send character, +/- , and the number of revolutions
    ##########
    def send_single_cw(self):
        self.status_label.config(text="Sent Single CW Command", fg="blue")
        speed = self.get_disc_speed()
        ser.write(str(speed + "+1" + "\n").encode())

    def send_single_ccw(self):
        self.status_label.config(text="Sent Single CCW Command", fg="blue")
        speed = self.get_disc_speed()
        ser.write(str(speed + "-1" + "\n").encode())

    def send_N_cw(self):
        N = str(self.cw_slider.get())
        self.status_label.config(text=f"Sent {N} CW Commands", fg="blue")
        speed = self.get_disc_speed()
        ser.write(str(speed + "+" + N + "\n").encode())

    def send_M_ccw(self):
        M = str(self.ccw_slider.get())
        self.status_label.config(text=f"Sent {M} CCW Commands", fg="blue")
        speed = self.get_disc_speed()
        ser.write(str(speed + "-" + M + "\n").encode())

# -------------------------------------
# ----  # Functions for showing errors
# -------------------------------------
    def show_error(self, message):
        self.status_label.config(text=f"Error: {message}", fg="red")

    def clear_error(self):
        self.status_label.config(
            text="Status: Ready                   Errors: None", fg="green")

# -------------------------------------
# ----  # Function to kill everything when GUI exits
# -------------------------------------
    def on_closing(self):
        self.running = False
        self.serial_thread_running = False

        if self.serial_thread is not None:
            self.serial_thread.join()
        self.window.destroy()

# Create and run the application
if __name__ == "__main__":
    window = tk.Tk()
    app = App(window)
    window.protocol("WM_DELETE_WINDOW", app.on_closing)
    window.mainloop()
