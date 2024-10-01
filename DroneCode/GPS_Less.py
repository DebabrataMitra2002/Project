import argparse
import tkinter as tk
from tkinter import messagebox
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
        master.title("Drone Control Center")
        master.geometry("600x400")

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
        arm_button = tk.Button(self.master, text="ARM", command=self.arm_drone, bg="green", fg="white", width=15)
        arm_button.pack(pady=10)

        disarm_button = tk.Button(self.master, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=15)
        disarm_button.pack(pady=10)

        rtl_button = tk.Button(self.master, text="RTL (Return to Launch)", command=self.set_rtl_mode, bg="blue", fg="white", width=25)
        rtl_button.pack(pady=10)

        loiter_button = tk.Button(self.master, text="Loiter Mode", command=self.set_loiter_mode, bg="orange", fg="white", width=25)
        loiter_button.pack(pady=10)

        land_button = tk.Button(self.master, text="Land Mode", command=self.land_drone, bg="brown", fg="white", width=25)
        land_button.pack(pady=10)

        non_gps_rtl_button = tk.Button(self.master, text="Non-GPS RTL", command=self.non_gps_rtl, bg="purple", fg="white", width=25)
        non_gps_rtl_button.pack(pady=10)

        # Status Display
        self.status_labels = {}
        status_items = ["Connection Status:", "GPS Status:", "Battery Level:", "Current Mode:", "Drone Location:"]
        for item in status_items:
            label = tk.Label(self.master, text=item)
            label.pack()
            value = tk.Label(self.master, text="N/A")
            value.pack()
            self.status_labels[item] = value

    def arm_drone(self):
        if self.vehicle.is_armable:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            while not self.vehicle.armed:
                time.sleep(1)
            messagebox.showinfo("Info", "Drone armed!")
        else:
            messagebox.showwarning("Warning", "Drone is not armable. Check GPS and battery.")

    def disarm_drone(self):
        self.vehicle.armed = False
        while self.vehicle.armed:
            time.sleep(1)
        messagebox.showinfo("Info", "Drone disarmed!")

    def set_rtl_mode(self):
        self.vehicle.mode = VehicleMode("RTL")
        messagebox.showinfo("Info", "Switching to RTL (Return to Launch) mode.")

    def set_loiter_mode(self):
        self.vehicle.mode = VehicleMode("LOITER")
        messagebox.showinfo("Info", "Switching to LOITER mode.")

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        messagebox.showinfo("Info", "Switching to LAND mode.")

    def non_gps_rtl(self):
        # Reverse the stored path log for Non-GPS RTL
        if path_log:
            messagebox.showinfo("Info", "Non-GPS RTL activated.")
            for location in reversed(path_log):
                self.vehicle.simple_goto(location)
                time.sleep(2)
        else:
            messagebox.showwarning("Warning", "No path data available for Non-GPS RTL.")

    def track_position(self):
        while True:
            # Record current drone position for path tracking
            if self.vehicle.location.global_frame:
                location = self.vehicle.location.global_frame
                path_log.append(LocationGlobalRelative(location.lat, location.lon, location.alt))
            time.sleep(5)

    def update_status(self):
        # Update status labels
        self.status_labels["Connection Status:"].config(text="Connected" if self.vehicle else "Disconnected")
        self.status_labels["GPS Status:"].config(text=str(self.vehicle.gps_0.fix_type))
        self.status_labels["Battery Level:"].config(text=str(self.vehicle.battery.level) + "%")
        self.status_labels["Current Mode:"].config(text=self.vehicle.mode.name)
        if self.vehicle.location.global_frame:
            location = self.vehicle.location.global_frame
            self.status_labels["Drone Location:"].config(text=f"Lat: {location.lat}, Lon: {location.lon}")
        else:
            self.status_labels["Drone Location:"].config(text="N/A")
        
        # Schedule next status update
        self.master.after(1000, self.update_status)

# Main function to launch the GUI
def main():
    root = tk.Tk()
    app = DroneControlGUI(root, vehicle)
    root.mainloop()

if __name__ == "__main__":
    main()


