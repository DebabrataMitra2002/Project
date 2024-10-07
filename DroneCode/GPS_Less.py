import argparse
import tkinter as tk
from tkinter import messagebox, ttk
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading

# Add argparse to parse command-line arguments for vehicle connection
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
args = parser.parse_args()

# Connect to the Vehicle using the argument from the command line
print("Connecting to vehicle on: %s" % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Global path storage for non-GPS RTL
path_log = []

# PID controller class (for altitude or other potential features)
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def calculate(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Main GUI class
class DroneControlGUI:
    def __init__(self, master, vehicle):
        self.master = master
        self.vehicle = vehicle
        master.title("SAFE FLY")  # Title of the interface
        master.geometry("700x500")

        # Status Bar
        self.status_bar = tk.Label(master, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.TOP, fill=tk.X)

        # Create the UI layout
        self.create_widgets()

        # Start tracking the drone status
        self.update_status()

        # Thread for tracking the drone's position
        self.tracking_thread = threading.Thread(target=self.track_position)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()

    def create_widgets(self):
        # Control Buttons
        control_frame = tk.Frame(self.master)
        control_frame.pack(pady=10)

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

        non_gps_rtl_button = tk.Button(control_frame, text="Non-GPS RTL", command=self.non_gps_rtl, bg="purple", fg="white", width=15)
        non_gps_rtl_button.grid(row=4, column=0, columnspan=2, pady=5)

        # Add Alt Hold and Stabilize Mode buttons
        alt_hold_button = tk.Button(control_frame, text="Alt Hold Mode", command=self.set_alt_hold_mode, bg="pink", fg="black", width=15)
        alt_hold_button.grid(row=5, column=0, columnspan=2, pady=5)

        stabilize_button = tk.Button(control_frame, text="Stabilize Mode", command=self.set_stabilize_mode, bg="cyan", fg="black", width=15)
        stabilize_button.grid(row=6, column=0, columnspan=2, pady=5)

        # Takeoff Button and Slider
        takeoff_frame = tk.Frame(self.master)
        takeoff_frame.pack(pady=20)
        
        self.altitude_slider = tk.Scale(takeoff_frame, from_=0, to=70, orient=tk.HORIZONTAL, label="Takeoff Altitude (m)", length=300)
        self.altitude_slider.set(0)  # Default altitude 0m
        self.altitude_slider.pack(side=tk.LEFT, padx=10)

        takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=10)
        takeoff_button.pack(side=tk.LEFT)

        # Go to Specific Location
        location_frame = tk.Frame(self.master)
        location_frame.pack(pady=10)

        tk.Label(location_frame, text="Lat:").grid(row=0, column=0)
        self.lat_entry = tk.Entry(location_frame)
        self.lat_entry.grid(row=0, column=1)

        tk.Label(location_frame, text="Lon:").grid(row=1, column=0)
        self.lon_entry = tk.Entry(location_frame)
        self.lon_entry.grid(row=1, column=1)

        tk.Label(location_frame, text="Alt:").grid(row=2, column=0)
        self.alt_entry = tk.Entry(location_frame)
        self.alt_entry.grid(row=2, column=1)

        # Move the Go to Location button to the right side of the entries
        go_button = tk.Button(location_frame, text="Go to Location", command=self.go_to_location, bg="lime", width=15)
        go_button.grid(row=3, column=2, padx=5, pady=5)

        # Status Display
        status_frame = tk.Frame(self.master)
        status_frame.pack(pady=10)

        self.status_labels = {}
        status_items = [
            "Connection Status:",
            "Current Mode:",
            "Drone Location:",
            "Altitude (m):"
        ]
        for item in status_items:
            label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
            label.pack(anchor="w")
            value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
            value.pack(anchor="w")
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
        self.status_bar.config(text="Status: Switching to RTL mode")

    def set_loiter_mode(self):
        self.vehicle.mode = VehicleMode("LOITER")
        self.show_message("Info", "Switching to LOITER mode.")
        self.status_bar.config(text="Status: Switching to LOITER mode")

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        self.show_message("Info", "Switching to LAND mode.")
        self.status_bar.config(text="Status: Switching to LAND mode")

    def non_gps_rtl(self):
        # Reverse the stored path log for Non-GPS RTL
        if path_log:
            self.show_message("Info", "Non-GPS RTL activated.")
            self.status_bar.config(text="Status: Non-GPS RTL activated")
            for location in reversed(path_log):
                self.vehicle.simple_goto(location)
                time.sleep(2)
        else:
            self.show_message("Warning", "No path data available for Non-GPS RTL.")
            self.status_bar.config(text="Status: No path data for Non-GPS RTL")

    def set_alt_hold_mode(self):
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        self.show_message("Info", "Switching to Alt Hold mode.")
        self.status_bar.config(text="Status: Switching to Alt Hold mode")

    def set_stabilize_mode(self):
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.show_message("Info", "Switching to Stabilize mode.")
        self.status_bar.config(text="Status: Switching to Stabilize mode")

    def takeoff_drone(self):
        altitude = self.altitude_slider.get()
        if self.vehicle.is_armable:
            self.vehicle.simple_takeoff(altitude)
            self.show_message("Info", f"Taking off to {altitude} meters!")
            self.status_bar.config(text=f"Status: Taking off to {altitude} meters!")
        else:
            self.show_message("Warning", "Drone is not armable. Check GPS and battery.")
            self.status_bar.config(text="Status: Drone Not Armable")

    def go_to_location(self):
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())
            alt = float(self.alt_entry.get())
            target_location = LocationGlobalRelative(lat, lon, alt)
            self.vehicle.simple_goto(target_location)
            self.show_message("Info", f"Going to Location: ({lat}, {lon}, {alt})")
            self.status_bar.config(text=f"Status: Going to Location: ({lat}, {lon}, {alt})")
            path_log.append(target_location)  # Log path for non-GPS RTL
        except ValueError:
            self.show_message("Error", "Invalid coordinates entered.")

    def update_status(self):
        # Update status every 1000ms
        if self.vehicle.mode.name:
            self.status_labels["Current Mode:"].config(text=self.vehicle.mode.name)
        if self.vehicle.location.global_relative_frame:
            self.status_labels["Drone Location:"].config(text=f"{self.vehicle.location.global_relative_frame.lat}, {self.vehicle.location.global_relative_frame.lon}")
        if self.vehicle.location.global_relative_frame.alt is not None:
            self.status_labels["Altitude (m):"].config(text=f"{self.vehicle.location.global_relative_frame.alt}")

        self.master.after(1000, self.update_status)

    def track_position(self):
        while True:
            time.sleep(1)  # Run tracking logic every second

    def show_message(self, title, message):
        messagebox.showinfo(title, message)

# Initialize and run the GUI
if __name__ == "__main__":
    root = tk.Tk()
    app = DroneControlGUI(root, vehicle)
    root.mainloop()

    # Close the vehicle connection
    vehicle.close()
