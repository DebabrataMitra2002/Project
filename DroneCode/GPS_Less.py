import argparse
import tkinter as tk
from tkinter import messagebox
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading

# Parse command-line arguments for vehicle connection
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550', help='Vehicle connection string')
args = parser.parse_args()

# Connect to the vehicle
print(f"Connecting to vehicle on: {args.connect}")
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Global path storage for non-GPS RTL
path_log = []

# PID Controller Class for future use
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

# Main GUI Class
class DroneControlGUI:
    def __init__(self, master, vehicle):
        self.master = master
        self.vehicle = vehicle
        master.title("Drone Control Center")
        master.geometry("600x500")  # Smaller screen size

        # Create the UI components
        self.create_widgets()

        # Start tracking drone status
        self.update_status()

        # Start position tracking in a separate thread
        self.tracking_thread = threading.Thread(target=self.track_position)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()

    def create_widgets(self):
        # Control Buttons
        control_frame = tk.Frame(self.master)
        control_frame.pack(pady=10)

        arm_button = tk.Button(control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", width=15)
        arm_button.grid(row=0, column=0, padx=10, pady=5)

        disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=15)
        disarm_button.grid(row=0, column=1, padx=10, pady=5)

        rtl_button = tk.Button(control_frame, text="RTL", command=self.set_rtl_mode, bg="blue", fg="white", width=15)
        rtl_button.grid(row=1, column=0, padx=10, pady=5)

        loiter_button = tk.Button(control_frame, text="Loiter Mode", command=self.set_loiter_mode, bg="orange", fg="white", width=15)
        loiter_button.grid(row=1, column=1, padx=10, pady=5)

        land_button = tk.Button(control_frame, text="Land Mode", command=self.land_drone, bg="brown", fg="white", width=15)
        land_button.grid(row=2, column=0, padx=10, pady=5)

        non_gps_rtl_button = tk.Button(control_frame, text="Non-GPS RTL", command=self.non_gps_rtl, bg="purple", fg="white", width=15)
        non_gps_rtl_button.grid(row=2, column=1, padx=10, pady=5)

        # New Buttons: Alt Hold and Brake Mode
        alt_hold_button = tk.Button(control_frame, text="Alt Hold Mode", command=self.set_alt_hold_mode, bg="yellow", fg="black", width=15)
        alt_hold_button.grid(row=3, column=0, padx=10, pady=5)

        brake_button = tk.Button(control_frame, text="Brake Mode", command=self.set_brake_mode, bg="darkred", fg="white", width=15)
        brake_button.grid(row=3, column=1, padx=10, pady=5)

        # Takeoff Button and Slider
        takeoff_frame = tk.Frame(self.master)
        takeoff_frame.pack(pady=20)

        self.altitude_slider = tk.Scale(takeoff_frame, from_=10, to=60, orient=tk.HORIZONTAL, label="Takeoff Altitude (m)", length=300)
        self.altitude_slider.set(10)  # Default altitude 10m
        self.altitude_slider.pack(side=tk.LEFT, padx=10)

        takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=15)
        takeoff_button.pack(side=tk.LEFT, padx=10)

        # New Section: Go to Specific Location (Lat, Lon, Alt)
        location_frame = tk.Frame(self.master)
        location_frame.pack(pady=10)

        lat_label = tk.Label(location_frame, text="Latitude:")
        lat_label.grid(row=0, column=0)
        self.lat_entry = tk.Entry(location_frame)
        self.lat_entry.grid(row=0, column=1)

        lon_label = tk.Label(location_frame, text="Longitude:")
        lon_label.grid(row=1, column=0)
        self.lon_entry = tk.Entry(location_frame)
        self.lon_entry.grid(row=1, column=1)

        alt_label = tk.Label(location_frame, text="Altitude (m):")
        alt_label.grid(row=2, column=0)
        self.alt_entry = tk.Entry(location_frame)
        self.alt_entry.grid(row=2, column=1)

        goto_button = tk.Button(location_frame, text="Go to Location", command=self.goto_location, bg="lightblue", width=15)
        goto_button.grid(row=3, column=0, columnspan=2, pady=10)

        # Status Display
        status_frame = tk.Frame(self.master)
        status_frame.pack(pady=10)

        self.status_labels = {}
        status_items = ["Connection Status:", "GPS Status:", "Battery Level:", "Current Mode:", "Drone Location:"]
        for item in status_items:
            label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
            label.pack(anchor="w")
            value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
            value.pack(anchor="w")
            self.status_labels[item] = value

        # Status Bar
        self.status_bar = tk.Label(self.master, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def arm_drone(self):
        if self.vehicle.is_armable:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            while not self.vehicle.armed:
                time.sleep(1)
            messagebox.showinfo("Info", "Drone armed!")
            self.status_bar.config(text="Status: Drone Armed")
        else:
            messagebox.showwarning("Warning", "Drone is not armable. Check GPS and battery.")
            self.status_bar.config(text="Status: Drone Not Armable")

    def disarm_drone(self):
        self.vehicle.armed = False
        while self.vehicle.armed:
            time.sleep(1)
        messagebox.showinfo("Info", "Drone disarmed!")
        self.status_bar.config(text="Status: Drone Disarmed")

    def set_rtl_mode(self):
        self.vehicle.mode = VehicleMode("RTL")
        messagebox.showinfo("Info", "Switching to RTL (Return to Launch) mode.")
        self.status_bar.config(text="Status: Switching to RTL mode")

    def set_loiter_mode(self):
        self.vehicle.mode = VehicleMode("LOITER")
        messagebox.showinfo("Info", "Switching to LOITER mode.")
        self.status_bar.config(text="Status: Switching to LOITER mode")

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        messagebox.showinfo("Info", "Switching to LAND mode.")
        self.status_bar.config(text="Status: Switching to LAND mode")

    def non_gps_rtl(self):
        if path_log:
            messagebox.showinfo("Info", "Non-GPS RTL activated.")
            self.status_bar.config(text="Status: Non-GPS RTL activated")
            for location in reversed(path_log):
                self.vehicle.simple_goto(location)
                time.sleep(2)
        else:
            messagebox.showwarning("Warning", "No path data available for Non-GPS RTL.")
            self.status_bar.config(text="Status: No path data for Non-GPS RTL")

    def set_alt_hold_mode(self):
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        messagebox.showinfo("Info", "Switching to Alt Hold mode.")
        self.status_bar.config(text="Status: Switching to Alt Hold mode")

    def set_brake_mode(self):
        self.vehicle.mode = VehicleMode("BRAKE")
        messagebox.showinfo("Info", "Activating Brake mode.")
        self.status_bar.config(text="Status: Activating Brake mode")

    def goto_location(self):
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())
            alt = float(self.alt_entry.get())
            target_location = LocationGlobalRelative(lat, lon, alt)
            self.vehicle.simple_goto(target_location)
            messagebox.showinfo("Info", f"Navigating to ({lat}, {lon}, {alt})")
            self.status_bar.config(text=f"Status: Navigating to ({lat}, {lon}, {alt})")
        except ValueError:
            messagebox.showwarning("Warning", "Invalid latitude, longitude, or altitude input.")
            self.status_bar.config(text="Status: Invalid Location Input")

    def takeoff_drone(self):
        altitude = self.altitude_slider.get()
        if self.vehicle.is_armable:
            self.vehicle.simple_takeoff(altitude)
            messagebox.showinfo("Info", f"Taking off to {altitude} meters!")
            self.status_bar.config(text=f"Status: Taking off to {altitude} meters")
        else:
            messagebox.showwarning("Warning", "Drone is not armable. Check GPS and battery.")
            self.status_bar.config(text="Status: Unable to takeoff. Drone is not armable.")

    def track_position(self):
        while True:
            if self.vehicle.location.global_frame:
                location = self.vehicle.location.global_frame
                path_log.append(LocationGlobalRelative(location.lat, location.lon, location.alt))
            time.sleep(5)

    def update_status(self):
        self.status_labels["Connection Status:"].config(text="Connected" if self.vehicle else "Disconnected")
        self.status_labels["GPS Status:"].config(text=str(self.vehicle.gps_0.fix_type))
        self.status_labels["Battery Level:"].config(text=str(self.vehicle.battery.level) + "%")
        self.status_labels["Current Mode:"].config(text=self.vehicle.mode.name)
        if self.vehicle.location.global_frame:
            location = self.vehicle.location.global_frame
            self.status_labels["Drone Location:"].config(text=f"Lat: {location.lat}, Lon: {location.lon}")
        else:
            self.status_labels["Drone Location:"].config(text="N/A")

        # Schedule the next status update
        self.master.after(1000, self.update_status)

# Main function to launch the GUI
def main():
    root = tk.Tk()
    app = DroneControlGUI(root, vehicle)
    root.mainloop()

if __name__ == "__main__":
    main()
