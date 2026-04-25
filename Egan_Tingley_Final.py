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
ser = serial.Serial('COM11', 115200, timeout=0)  # Update 'COM__' to the Arduino's port

class App(tk.Tk):

    # Initialize the GUI and set up the layout with buttons and labels
    def __init__(self, window):
        # Label the window and set its size
        self.window = window
        self.window.title("MCE 530 Final")
        self.window.geometry('1150x850')
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

    
        # Create update plot button for number of data points and updating x and y limits
        self.update_plot_config_button = tk.Button(plot_control_frame, text="Update Plot", command=self.update_plot_config)
        self.update_plot_config_button.grid(row=4, column=2, padx=10, pady=10)

        
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
        self.raw_voltage_label = tk.Label(feedback_frame, text="LiveVoltage Reading: N/A", fg="green")
        self.raw_voltage_label.place(x=10, y=210)  

        self.pwm_label = tk.Label(feedback_frame, text="Live PWM Sent: N/A", fg="green")
        self.pwm_label.place(x=10, y=260)  

# -------------------------------------
# ----  # Create status bar frame for most recent command sent and error messages
# -------------------------------------
        self.status_frame = tk.LabelFrame(window, text="Status and Most Recent Commands", padx=10, pady=10)
        self.status_frame.place(x=320, y=480, width=350, height=80)

        self.recent_sent_status_label = tk.Label(self.status_frame, text="Status: Ready", fg="green")
        self.recent_sent_status_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")

        self.error_label = tk.Label(self.status_frame, text="Errors: None", fg="green")
        self.error_label.grid(row=0, column=1, padx=100, pady=5, sticky="w")

# -------------------------------------
# ----  # Create frame for controlling the DC motor trajectory (3 sections: each with starting V, ending V, and duration)
# -------------------------------------
        self.velocity_trajectory_frame = tk.LabelFrame(window, text="DC Motor Trajectory Controls", padx=10, pady=10, fg="orange")
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
        self.send_trajectory_button.place(x=200, y=170)

        self.preview_trajectory_button = tk.Button(self.velocity_trajectory_frame, text="Preview Trajectory", command=self.preview_trajectory)
        self.preview_trajectory_button.place(x=50, y=170)        

# -------------------------------------
# ----  # Create frame for controlling the stepper motor disc and displaying the net revolutions
# -------------------------------------
        self.disk_control_frame = tk.LabelFrame(window, text="Stepper Motor Disc Controls", padx=10, pady=10, fg="blue")
        self.disk_control_frame.place(x=50, y=480, width=240, height=350)

        # Buttons to send single CW or single CCW
        self.cw_button = tk.Button(self.disk_control_frame, text="Single CW", command=self.send_single_cw)
        self.cw_button.grid(row=1, column=0, padx=10, pady=10)
        self.ccw_button = tk.Button(self.disk_control_frame, text="Single CCW", command=self.send_single_ccw)
        self.ccw_button.grid(row=1, column=1, padx=10, pady=10)

        # Slider for selecting N CW commands (1-10) and send button
        self.cw_slider = tk.Scale(self.disk_control_frame, from_=1, to=10, orient=tk.HORIZONTAL)
        self.cw_slider.grid(row=2, column=0, padx=5, pady=5)
        self.cw_send_button = tk.Button(self.disk_control_frame, text="Send N CW", command=self.send_N_cw)
        self.cw_send_button.grid(row=2, column=1, padx=10, pady=20)
        self.N_cw_label = tk.Label(self.disk_control_frame, text="Set N CW")
        self.N_cw_label.grid(row=3, column=0, padx=10, pady=10)

        # Slider for selecting M CCW commands (1-10) and send button
        self.ccw_slider = tk.Scale(self.disk_control_frame, from_=1, to=10, orient=tk.HORIZONTAL)
        self.ccw_slider.grid(row=4, column=0, padx=10, pady=10)
        self.ccw_send_button = tk.Button(self.disk_control_frame, text="Send M CCW", command=self.send_M_ccw)
        self.ccw_send_button.grid(row=4, column=1, padx=10, pady=20)
        self.M_ccw_label = tk.Label(self.disk_control_frame, text="Set M CCW")
        self.M_ccw_label.grid(row=5, column=0, padx=10, pady=10)

        # Stop button
        self.stop_disc_button = tk.Button(self.disk_control_frame, text="Stop Disc", command=self.stop_disc_motion)
        self.stop_disc_button.grid(row=6, column=0, padx=10, pady=10)

        # Label to display net revolutions, this will be updated live from the serial output of the arduino
        self.net_rev_label = tk.Label(self.disk_control_frame, text="Net Revs: 0", fg="blue")
        self.net_rev_label.grid(row=6, column=1, padx=10, pady=10)

# -------------------------------------
# Function to create the plot based on user settings
# -------------------------------------
    def create_plot(self):
        # Data storage
        self.max_points = int(self.num_data_points_entry.get())

        # Set up data deques
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)

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

# create empty functions to test GUI spacing and button functionality before adding in the serial communication and plotting code
    def select_simulation_mode(self):
        self.recent_sent_status_label.config(text="Status: Selected Simulation Mode", fg="green")
        self.simulation_mode_button.config(state=tk.DISABLED)
        self.real_mode_button.config(state=tk.NORMAL)
    
    def select_real_lab_setup_mode(self):
        self.recent_sent_status_label.config(text="Status: Selected Real Lab Mode", fg="green")
        self.simulation_mode_button.config(state=tk.NORMAL)
        self.real_mode_button.config(state=tk.DISABLED)
    
    def start_plot(self):
        self.recent_sent_status_label.config(text="Status: Started Plotting", fg="green")
        self.start_button.config(state=tk.DISABLED)
        self.simulation_mode_button.config(state=tk.DISABLED)
        self.real_mode_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

    def stop_plot(self):
        self.recent_sent_status_label.config(text="Status: Stopped Plotting", fg="green")
        self.start_button.config(state=tk.NORMAL)
        self.simulation_mode_button.config(state=tk.NORMAL)
        self.real_mode_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def clear_plot(self):
        self.recent_sent_status_label.config(text="Status: Cleared Plot", fg="green")

    def send_interval(self):
        self.recent_sent_status_label.config(text=f"Status: Sent Interval {self.sample_interval_entry.get()}s", fg="green")

    def send_duration(self):
        self.recent_sent_status_label.config(text=f"Status: Sent Duration {self.sample_duration_entry.get()}s", fg="green")

    def update_plot_config(self):
        self.max_points = int(self.num_data_points_entry.get())
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        self.ax.set_xlim(0, self.max_points)
        self.ax.set_ylim(*self.get_y_limits())
        self.canvas.draw()
        self.recent_sent_status_label.config(text="Status: Updated Plot Config", fg="green")
    
    def get_y_limits(self):
        try:
            y_min = float(self.y_min_entry.get())
            y_max = float(self.y_max_entry.get())
            if y_min >= y_max:
                raise ValueError("Y-Min must be less than Y-Max.")
            return y_min, y_max
        except ValueError as e:
            self.error_label.config(text=f"Errors: {str(e)}", fg="red")
            return 0, 6  # Return default limits if there's an error
        
    def send_step_input(self):
        self.recent_sent_status_label.config(text=f"Status: Sent Step Input {self.step_input_entry.get()} V", fg="green")
    
    def send_kp(self):
        self.recent_sent_status_label.config(text=f"Status: Sent Kp Gain {self.kp_entry.get()}", fg="green")

    def send_ki(self):
        self.recent_sent_status_label.config(text=f"Status: Sent Ki Gain {self.ki_entry.get()}", fg="green")
    
    def remove_feedback(self):
        self.recent_sent_status_label.config(text="Status: Removed Feedback (Open Loop)", fg="green")

    def preview_trajectory(self):
        self.recent_sent_status_label.config(text="Status: Previewed Trajectory", fg="green")

    def send_trajectory(self):
        self.recent_sent_status_label.config(text="Status: Sent Trajectory", fg="green")

    def send_single_cw(self):
        self.recent_sent_status_label.config(text="Status: Sent Single CW Command", fg="green")
    
    def send_single_ccw(self):
        self.recent_sent_status_label.config(text="Status: Sent Single CCW Command", fg="green")

    def send_N_cw(self):
        N = self.cw_slider.get()
        self.recent_sent_status_label.config(text=f"Status: Sent {N} CW Commands", fg="green")
    
    def send_M_ccw(self):
        M = self.ccw_slider.get()
        self.recent_sent_status_label.config(text=f"Status: Sent {M} CCW Commands", fg="green")

    def stop_disc_motion(self):
        self.recent_sent_status_label.config(text="Status: Sent Stop Command for Disc", fg="green")

# main function to run the app
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    app.create_plot()
    root.mainloop()