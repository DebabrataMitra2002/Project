import tkinter as tk
from tkinter import messagebox
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import time
import folium
import os
import threading
import requests

# Constants
CHROME_DRIVER_PATH = '/path/to/chromedriver'  # Update this with the path to ChromeDriver
MAP_HTML = "drone_map.html"
UPDATE_INTERVAL = 1 # seconds
MAP_SERVER_URL = "http://localhost:5000/update_location"

class DroneControlGUI:
    def __init__(self, master, vehicle):
        self.master = master
        master.title("Drone Control Center with Real-Time Map")
        master.configure(bg="lightblue")
        master.geometry("600x500")
        
        # Initialize vehicle connection
        self.vehicle = vehicle
        
        # Create frames for better layout management
        self.create_frames()
        
        # Create widgets
        self.create_widgets()
        
        # Start tracking drone status
        self.update_status()
        
        # Start tracking drone location in a separate thread
        self.tracking_thread = threading.Thread(target=self.track_drone)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
    
    def create_frames(self):
        # Frame for Inputs
        self.input_frame = tk.Frame(self.master, bg="lightblue")
        self.input_frame.pack(pady=10)
        
        # Frame for Control Buttons
        self.control_frame = tk.Frame(self.master, bg="lightblue")
        self.control_frame.pack(pady=10)
        
        # Frame for Map Button
        self.map_frame = tk.Frame(self.master, bg="lightblue")
        self.map_frame.pack(pady=10)
        
        # Frame for Status Bar
        self.status_frame = tk.Frame(self.master, bg="lightgray")
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X)
    
    def create_widgets(self):
        # Title label
        title_label = tk.Label(self.input_frame, text="Drone Control Center", font=("Helvetica", 16, "bold"), bg="lightblue", fg="darkblue")
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Latitude Label and Entry
        label_latitude = tk.Label(self.input_frame, text="Latitude:", bg="lightblue", font=("Helvetica", 12))
        label_latitude.grid(row=1, column=0, sticky="e", padx=10, pady=5)
        self.entry_latitude = tk.Entry(self.input_frame, width=20)
        self.entry_latitude.grid(row=1, column=1, pady=5)
        
        # Longitude Label and Entry
        label_longitude = tk.Label(self.input_frame, text="Longitude:", bg="lightblue", font=("Helvetica", 12))
        label_longitude.grid(row=2, column=0, sticky="e", padx=10, pady=5)
        self.entry_longitude = tk.Entry(self.input_frame, width=20)
        self.entry_longitude.grid(row=2, column=1, pady=5)
        
        # Altitude Label and Entry
        label_altitude = tk.Label(self.input_frame, text="Altitude (m):", bg="lightblue", font=("Helvetica", 12))
        label_altitude.grid(row=3, column=0, sticky="e", padx=10, pady=5)
        self.entry_altitude = tk.Entry(self.input_frame, width=20)
        self.entry_altitude.grid(row=3, column=1, pady=5)
        
        # Send Drone Button
        send_button = tk.Button(self.input_frame, text="Send Drone", command=self.send_drone_to_location, bg="darkblue", fg="white", font=("Helvetica", 12), width=15)
        send_button.grid(row=4, column=0, columnspan=2, pady=10)
        
        # Control Buttons
        arm_button = tk.Button(self.control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", font=("Helvetica", 12), width=10)
        arm_button.grid(row=0, column=0, padx=5, pady=5)
        
        disarm_button = tk.Button(self.control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", font=("Helvetica", 12), width=10)
        disarm_button.grid(row=0, column=1, padx=5, pady=5)
        
        loiter_button = tk.Button(self.control_frame, text="Loiter", command=self.set_loiter_mode, bg="orange", fg="white", font=("Helvetica", 12), width=10)
        loiter_button.grid(row=1, column=0, padx=5, pady=5)
        
        rtl_button = tk.Button(self.control_frame, text="RTL", command=self.set_rtl_mode, bg="purple", fg="white", font=("Helvetica", 12), width=10)
        rtl_button.grid(row=1, column=1, padx=5, pady=5)
        
        # Open Map Button
        map_button = tk.Button(self.map_frame, text="Open Real-Time Map", command=self.open_map, bg="darkblue", fg="white", font=("Helvetica", 12), width=20)
        map_button.pack(pady=10)
        
        # Status Bar Labels
        self.status_labels = {}
        status_items = ["Connection Status:", "GPS Status:", "Battery Level:", "Current Mode:", "Drone Location:"]
        for idx, item in enumerate(status_items):
            label = tk.Label(self.status_frame, text=item, bg="lightgray", font=("Helvetica", 10, "bold"))
            label.grid(row=0, column=idx*2, sticky="e", padx=5, pady=2)
            value = tk.Label(self.status_frame, text="N/A", bg="lightgray", font=("Helvetica", 10))
            value.grid(row=0, column=idx*2+1, sticky="w", padx=5, pady=2)
            self.status_labels[item] = value
    
    def send_drone_to_location(self):
        try:
            lat = float(self.entry_latitude.get())
            lon = float(self.entry_longitude.get())
            alt = float(self.entry_altitude.get())
            
            # Safety Checks
            if not self.vehicle.is_armable:
                messagebox.showwarning("Warning", "Vehicle is not armable. Check GPS and battery.")
                return
            
            if self.vehicle.battery is None or self.vehicle.battery.level < 20:
                messagebox.showwarning("Warning", "Battery level too low!")
                return
            
            if alt == 0:
                # Land at current location
                current_location = self.vehicle.location.global_frame
                target_location = LocationGlobalRelative(current_location.lat, current_location.lon, 0)
                self.vehicle.simple_goto(target_location)
            else:
                target_location = LocationGlobalRelative(lat, lon, alt)
                self.vehicle.simple_goto(target_location)
            
            messagebox.showinfo("Info", f"Drone is moving to Latitude: {lat}, Longitude: {lon}, Altitude: {alt}m")
        
        except ValueError:
            messagebox.showerror("Error", "Invalid input. Please enter valid numbers for latitude, longitude, and altitude.")
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")
    
    def arm_drone(self):
        try:
            if not self.vehicle.is_armable:
                messagebox.showwarning("Warning", "Vehicle is not armable. Check GPS and battery.")
                return
            
            if self.vehicle.battery is None or self.vehicle.battery.level < 20:
                messagebox.showwarning("Warning", "Battery level too low!")
                return
            
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            messagebox.showinfo("Info", "Arming drone...")
            
            # Wait until the drone is armed
            while not self.vehicle.armed:
                time.sleep(1)
            messagebox.showinfo("Info", "Drone armed!")
        
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred while arming: {e}")
    
    def disarm_drone(self):
        try:
            self.vehicle.armed = False
            messagebox.showinfo("Info", "Disarming drone...")
            
            # Wait until the drone is disarmed
            while self.vehicle.armed:
                time.sleep(1)
            messagebox.showinfo("Info", "Drone disarmed!")
        
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred while disarming: {e}")
    
    def set_loiter_mode(self):
        try:
            self.vehicle.mode = VehicleMode("LOITER")
            messagebox.showinfo("Info", "Switching to LOITER mode...")
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred while switching to LOITER: {e}")
    
    def set_rtl_mode(self):
        try:
            self.vehicle.mode = VehicleMode("RTL")
            messagebox.showinfo("Info", "Switching to RTL (Return to Launch) mode...")
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred while switching to RTL: {e}")
    
    def open_map(self):
        try:
            # Update map before opening
            location = self.vehicle.location.global_frame
            self.update_map(location.lat, location.lon)
            
            # Initialize Selenium WebDriver to open the map
            service = service(CHROME_DRIVER_PATH)
            driver = driver.Chrome(service=service)
            driver.get(f"file://{os.path.abspath(MAP_HTML)}")
            
            # Create an HTTP server for receiving location data
            server_thread = threading.Thread(target=self.run_server)
            server_thread.daemon = True
            server_thread.start()
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred while opening the map: {e}")
    
    def run_server(self):
        from server import app
        app.run(port=5000)
    
    def update_map(self, lat, lon):
        try:
            # Create a folium map centered on the drone's location
            drone_map = folium.Map(location=[lat, lon], zoom_start=15)
            folium.Marker([lat, lon], popup="Drone Location", icon=folium.Icon(color="blue")).add_to(drone_map)
            
            # Save map to HTML file
            drone_map.save(MAP_HTML)
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred while updating the map: {e}")
    
    def track_drone(self):
        while True:
            try:
                location = self.vehicle.location.global_frame
                self.update_map(location.lat, location.lon)
                time.sleep(UPDATE_INTERVAL)
            except Exception as e:
                print(f"An error occurred while tracking the drone: {e}")
    
    def update_status(self):
        # Update status labels
        self.status_labels["Connection Status:"].config(text="Connected" if self.vehicle else "Disconnected")
        self.status_labels["GPS Status:"].config(text=f"{self.vehicle.gps_0.fix_type}")
        self.status_labels["Battery Level:"].config(text=f"{self.vehicle.battery.level}%")
        self.status_labels["Current Mode:"].config(text=self.vehicle.mode.name)
        location = self.vehicle.location.global_frame
        self.status_labels["Drone Location:"].config(text=f"Lat: {location.lat}, Lon: {location.lon}")
        
        # Schedule next status update
        self.master.after(1000, self.update_status)

# Add argparse to parse command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print("Connecting to vehicle on: %s" % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Run the Drone Control GUI
root = tk.Tk()
app = DroneControlGUI(root, vehicle)
root.mainloop()
