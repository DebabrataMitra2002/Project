import argparse
import tkinter as tk
from tkinter import messagebox
from dronekit import connect, VehicleMode
import time
import threading
import RPi.GPIO as GPIO

# Add argparse to parse command-line arguments for vehicle connection
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
args = parser.parse_args()

# GPIO setup for HC-SR04 on multiple directions (front, back, left, right)
FRONT_TRIG, FRONT_ECHO = 23, 24
BACK_TRIG, BACK_ECHO = 25, 26
LEFT_TRIG, LEFT_ECHO = 27, 28
RIGHT_TRIG, RIGHT_ECHO = 29, 30

GPIO.setmode(GPIO.BCM)

# Setup for front sensor
GPIO.setup(FRONT_TRIG, GPIO.OUT)
GPIO.setup(FRONT_ECHO, GPIO.IN)

# Setup for back sensor
GPIO.setup(BACK_TRIG, GPIO.OUT)
GPIO.setup(BACK_ECHO, GPIO.IN)

# Setup for left sensor
GPIO.setup(LEFT_TRIG, GPIO.OUT)
GPIO.setup(LEFT_ECHO, GPIO.IN)

# Setup for right sensor
GPIO.setup(RIGHT_TRIG, GPIO.OUT)
GPIO.setup(RIGHT_ECHO, GPIO.IN)

# Function to measure distance using HC-SR04 sensor
def measure_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.2)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

class DroneControlGUI:
    def __init__(self, master, connection_string):
        self.master = master
        self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
        self.obstacle_prevent_enabled = False
        self.obstacle_prevent_distance = 2  # Default obstacle avoidance distance in meters

        master.title("SAFE FLY")
        master.geometry("800x600")

        # Status Bar
        self.status_bar = tk.Label(master, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.TOP, fill=tk.X)

        # Create the UI layout
        self.create_widgets()

        # Start tracking the drone status
        self.update_status()

        # Thread for tracking the drone's position and obstacle detection
        self.obstacle_thread = threading.Thread(target=self.obstacle_avoidance_system)
        self.obstacle_thread.daemon = True
        self.obstacle_thread.start()

    def create_widgets(self):
        # Main frame
        main_frame = tk.Frame(self.master)
        main_frame.pack(pady=10)

        # Control Buttons Frame
        control_frame = tk.Frame(main_frame)
        control_frame.pack(side=tk.LEFT, padx=20)

        # Control Buttons
        arm_button = tk.Button(control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", width=10)
        arm_button.grid(row=0, column=0, padx=5)

        disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=10)
        disarm_button.grid(row=0, column=1, padx=5)

        rtl_button = tk.Button(control_frame, text="RTL (Return to Launch)", command=self.set_rtl_mode, bg="blue", fg="white", width=15)
        rtl_button.grid(row=1, column=0, columnspan=2, pady=5)

        loiter_button = tk.Button(control_frame, text="Loiter Mode", command=self.set_loiter_mode, bg="orange", fg="white", width=15)
        loiter_button.grid(row=2, column=0, columnspan=2, pady=5)

        land_button = tk.Button(control_frame, text="Land Mode", command=self.land_drone, bg="brown", fg="white", width=15)
        land_button.grid(row=3, column=0, columnspan=2, pady=5)

        # Takeoff Button and Slider
        takeoff_frame = tk.Frame(control_frame)
        takeoff_frame.grid(row=4, column=0, columnspan=2, pady=20)

        self.altitude_slider = tk.Scale(takeoff_frame, from_=0, to=30, orient=tk.HORIZONTAL, label="Takeoff Altitude (m)", length=200)
        self.altitude_slider.set(0)  # Default altitude 0m
        self.altitude_slider.pack(side=tk.LEFT, padx=10)

        takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=10)
        takeoff_button.pack(side=tk.LEFT)

        # Obstacle Avoidance Slider
        self.obstacle_slider = tk.Scale(control_frame, from_=1, to=5, orient=tk.HORIZONTAL, label="Safe Distance (m)", length=200)
        self.obstacle_slider.set(2)  # Default 2 meters
        self.obstacle_slider.grid(row=5, column=0, columnspan=2, pady=5)

        # Enable/Disable Obstacle Avoidance
        self.avoidance_mode_button = tk.Button(control_frame, text="Enable Avoidance", command=self.toggle_avoidance_mode, bg="lightblue", width=15)
        self.avoidance_mode_button.grid(row=6, column=0, columnspan=2, pady=5)

        # Status Display
        status_frame = tk.Frame(main_frame)
        status_frame.pack(side=tk.LEFT, padx=20)

        self.status_labels = {}
        status_items = [
            "Connection Status:",
            "Current Mode:",
            "Drone Location:",
            "Altitude (m):",
            "Battery Level:",
            "Pitch:",
            "Roll:",
            "Yaw:",
            "Obstacle Status:"
        ]

        for i, item in enumerate(status_items):
            label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
            label.grid(row=i, column=0, sticky="w")
            value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
            value.grid(row=i, column=1, sticky="w")
            self.status_labels[item] = value

    def arm_drone(self):
        if self.vehicle.is_armable:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            while not self.vehicle.armed:
                time.sleep(1)
            self.show_message("Info", "Drone armed!")
            self.status_bar.config(text="Status: Drone Armed")
        else:
            self.show_message("Warning", "Drone is not armable. Check GPS and battery.")
            self.status_bar.config(text="Status: Drone Not Armable")

    def disarm_drone(self):
        self.vehicle.armed = False
        while self.vehicle.armed:
            time.sleep(1)
        self.show_message("Info", "Drone disarmed!")
        self.status_bar.config(text="Status: Drone Disarmed")

    def set_rtl_mode(self):
        self.vehicle.mode = VehicleMode("RTL")
        self.show_message("Info", "Switching to RTL (Return to Launch) mode.")
        self.status_bar.config(text="Status: RTL Mode")

    def set_loiter_mode(self):
        self.vehicle.mode = VehicleMode("LOITER")
        self.show_message("Info", "Switching to Loiter mode.")
        self.status_bar.config(text="Status: Loiter Mode")

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        self.show_message("Info", "Switching to Land mode.")
        self.status_bar.config(text="Status: Landing")

    def takeoff_drone(self):
        altitude = self.altitude_slider.get()
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.simple_takeoff(altitude)  # Take off to target altitude
        self.show_message("Info", f"Taking off to {altitude} meters.")

    def update_status(self):
        # Update GUI with drone status information
        self.status_labels["Connection Status:"].config(text="Connected" if self.vehicle.is_armable else "Disconnected")
        self.status_labels["Current Mode:"].config(text=self.vehicle.mode.name)
        self.status_labels["Drone Location:"].config(text=f"Lat: {self.vehicle.location.global_frame.lat}, Lon: {self.vehicle.location.global_frame.lon}")
        self.status_labels["Altitude (m):"].config(text=self.vehicle.location.global_frame.alt)
        self.status_labels["Battery Level:"].config(text=f"{self.vehicle.battery.level}%")
        self.status_labels["Pitch:"].config(text=f"{self.vehicle.attitude.pitch:.2f}")
        self.status_labels["Roll:"].config(text=f"{self.vehicle.attitude.roll:.2f}")
        self.status_labels["Yaw:"].config(text=f"{self.vehicle.attitude.yaw:.2f}")
        self.master.after(1000, self.update_status)  # Update every second

    def toggle_avoidance_mode(self):
        self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
        self.avoidance_mode_button.config(text="Disable Avoidance" if self.obstacle_prevent_enabled else "Enable Avoidance")
        self.show_message("Info", "Obstacle avoidance " + ("enabled." if self.obstacle_prevent_enabled else "disabled."))

    
    def obstacle_avoidance_system(self):
     while True:
        if self.obstacle_prevent_enabled:
            # Get safe distance from the slider
            self.obstacle_prevent_distance = self.obstacle_slider.get()

            # Measure distances from sensors
            front_distance = measure_distance(FRONT_TRIG, FRONT_ECHO)
            back_distance = measure_distance(BACK_TRIG, BACK_ECHO)
            left_distance = measure_distance(LEFT_TRIG, LEFT_ECHO)
            right_distance = measure_distance(RIGHT_TRIG, RIGHT_ECHO)

            # Get the drone's current velocity
            current_velocity = self.vehicle.velocity  # (vx, vy, vz) in m/s
            speed_magnitude = (current_velocity[0] ** 2 + current_velocity[1] ** 2 + current_velocity[2] ** 2) ** 0.5

            # Obstacle detection flags
            obstacle_front = front_distance < self.obstacle_prevent_distance
            obstacle_back = back_distance < self.obstacle_prevent_distance
            obstacle_left = left_distance < self.obstacle_prevent_distance
            obstacle_right = right_distance < self.obstacle_prevent_distance

            # Decision Logic for Obstacle Avoidance
            obstacle_status = "All Clear"

            # Increase height if obstacles are detected in front & back, left & right, or all four or three directions
            if (obstacle_front and obstacle_back) or (obstacle_left and obstacle_right) or \
               (obstacle_front and obstacle_back and obstacle_left) or \
               (obstacle_front and obstacle_back and obstacle_right) or \
               (obstacle_left and obstacle_right and obstacle_back) or \
               (obstacle_left and obstacle_right and obstacle_front) or \
               (obstacle_front and obstacle_back and obstacle_left and obstacle_right):
                self.vehicle.simple_goto(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt + 1)
                obstacle_status = "Obstacles in multiple directions - Increasing Height!"
            else:
                # Move backward or sideways depending on obstacle position
                if obstacle_front and not (obstacle_back or obstacle_left or obstacle_right):
                    if speed_magnitude > 2:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon - 2, self.vehicle.location.global_frame.alt)
                    else:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon - 1, self.vehicle.location.global_frame.alt)
                    obstacle_status = "Obstacle detected in Front - Moving Backward!"
                
                elif obstacle_back and not (obstacle_front or obstacle_left or obstacle_right):
                    if speed_magnitude > 2:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon + 2, self.vehicle.location.global_frame.alt)
                    else:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon + 1, self.vehicle.location.global_frame.alt)
                    obstacle_status = "Obstacle detected in Back - Moving Forward!"
                
                elif obstacle_left and not (obstacle_front or obstacle_back or obstacle_right):
                    if speed_magnitude > 2:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat + 2, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt)
                    else:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat + 1, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt)
                    obstacle_status = "Obstacle detected on Left - Moving Right!"
                
                elif obstacle_right and not (obstacle_front or obstacle_back or obstacle_left):
                    if speed_magnitude > 2:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat - 2, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt)
                    else:
                        self.vehicle.simple_goto(self.vehicle.location.global_frame.lat - 1, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt)
                    obstacle_status = "Obstacle detected on Right - Moving Left!"

            # If velocity is too low, stop and maintain safe distance
            if speed_magnitude < 2:
                self.vehicle.velocity = (0, 0, 0)
                obstacle_status += " | Low speed - Maintaining Safe Distance"

            # Update obstacle status in the GUI
            self.status_labels["Obstacle Status:"].config(text=obstacle_status)

        time.sleep(0.5)  # Wait half a second before the next check


    def show_message(self, title, message):
        messagebox.showinfo(title, message)

    def close(self):
        self.vehicle.close()
        GPIO.cleanup()
        self.master.quit()

# Main code execution
if __name__ == '__main__':
    root = tk.Tk()
    drone_gui = DroneControlGUI(root, args.connect)
    root.protocol("WM_DELETE_WINDOW", drone_gui.close)  # Properly close the vehicle on exit
    root.mainloop()
