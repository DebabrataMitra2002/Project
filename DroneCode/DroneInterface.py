import tkinter as tk
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import folium
import os
import threading
from selenium import webdriver
from selenium.webdriver.chrome.service import Service

# Function to create and update the Google Map
def update_map(lat, lon):
    drone_map = folium.Map(location=[lat, lon], zoom_start=15)
    folium.Marker([lat, lon], popup="Drone Location", icon=folium.Icon(color="red")).add_to(drone_map)
    drone_map.save("drone_map.html")

# Function to open the map in a web browser
def open_map():
    chrome_driver_path = '/path/to/chromedriver'  # Update this with the path to ChromeDriver
    service = Service(chrome_driver_path)
    browser = webdriver.Chrome(service=service)
    browser.get(f"file://{os.getcwd()}/drone_map.html")

# Connect to the vehicle (replace with your drone's connection string)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to track the drone's real-time location and update the map
def track_drone():
    while True:
        location = vehicle.location.global_frame
        lat, lon = location.lat, location.lon
        print(f"Current Drone Location -> Latitude: {lat}, Longitude: {lon}")
        update_map(lat, lon)
        time.sleep(5)

# Function to send the drone to the input coordinates
def send_drone_to_location():
    try:
        lat = float(entry_latitude.get())
        lon = float(entry_longitude.get())
        alt = float(entry_altitude.get())
        target_location = LocationGlobalRelative(lat, lon, alt)
        print(f"Sending drone to Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
        vehicle.simple_goto(target_location)
    except ValueError:
        print("Invalid input. Please enter valid numbers for latitude, longitude, and altitude.")

# Function to ARM the drone
def arm_drone():
    print("Arming drone...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)
    print("Drone armed!")

# Function to DISARM the drone
def disarm_drone():
    print("Disarming drone...")
    vehicle.armed = False
    while vehicle.armed:
        print("Waiting for vehicle to disarm...")
        time.sleep(1)
    print("Drone disarmed!")

# Function to set the drone to LOITER mode
def set_loiter_mode():
    print("Switching to LOITER mode...")
    vehicle.mode = VehicleMode("LOITER")

# Function to set the drone to RTL (Return to Launch) mode
def set_rtl_mode():
    print("Switching to RTL mode...")
    vehicle.mode = VehicleMode("RTL")

# Create the GUI window
window = tk.Tk()
window.title("Drone Control with Real-Time Map")

# Set background and general styling
window.configure(bg="lightblue")
window.geometry("400x300")

# Title label
title_label = tk.Label(window, text="Drone Control Center", font=("Helvetica", 16, "bold"), bg="lightblue", fg="darkblue")
title_label.grid(row=0, column=0, columnspan=2, pady=10)

# Latitude Label and Entry
label_latitude = tk.Label(window, text="Latitude:", bg="lightblue", font=("Helvetica", 12))
label_latitude.grid(row=1, column=0, sticky="e", padx=10, pady=5)
entry_latitude = tk.Entry(window)
entry_latitude.grid(row=1, column=1, pady=5)

# Longitude Label and Entry
label_longitude = tk.Label(window, text="Longitude:", bg="lightblue", font=("Helvetica", 12))
label_longitude.grid(row=2, column=0, sticky="e", padx=10, pady=5)
entry_longitude = tk.Entry(window)
entry_longitude.grid(row=2, column=1, pady=5)

# Altitude Label and Entry
label_altitude = tk.Label(window, text="Altitude:", bg="lightblue", font=("Helvetica", 12))
label_altitude.grid(row=3, column=0, sticky="e", padx=10, pady=5)
entry_altitude = tk.Entry(window)
entry_altitude.grid(row=3, column=1, pady=5)

# Button to send the drone to the input location
send_button = tk.Button(window, text="Send Drone", command=send_drone_to_location, bg="darkblue", fg="white", font=("Helvetica", 12))
send_button.grid(row=4, column=1, pady=5)

# Button for ARM
arm_button = tk.Button(window, text="ARM", command=arm_drone, bg="green", fg="white", font=("Helvetica", 12))
arm_button.grid(row=5, column=0, padx=10, pady=5)

# Button for DISARM
disarm_button = tk.Button(window, text="DISARM", command=disarm_drone, bg="red", fg="white", font=("Helvetica", 12))
disarm_button.grid(row=5, column=1, padx=10, pady=5)

# Button for LOITER mode
loiter_button = tk.Button(window, text="Loiter", command=set_loiter_mode, bg="orange", fg="white", font=("Helvetica", 12))
loiter_button.grid(row=6, column=0, padx=10, pady=5)

# Button for RTL mode
rtl_button = tk.Button(window, text="RTL", command=set_rtl_mode, bg="purple", fg="white", font=("Helvetica", 12))
rtl_button.grid(row=6, column=1, padx=10, pady=5)

# Button to open the map
map_button = tk.Button(window, text="Open Real-Time Map", command=open_map, bg="darkblue", fg="white", font=("Helvetica", 12))
map_button.grid(row=7, column=1, pady=10)

# Start a separate thread to track the drone's location in real time
tracking_thread = threading.Thread(target=track_drone)
tracking_thread.start()

# Start the GUI loop
window.mainloop()

# Close the vehicle connection
vehicle.close()
