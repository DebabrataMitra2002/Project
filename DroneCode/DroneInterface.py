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
    # Create a map centered at the drone's location
    drone_map = folium.Map(location=[lat, lon], zoom_start=15)

    # Add a marker for the drone's location
    folium.Marker([lat, lon], popup="Drone Location", icon=folium.Icon(color="red")).add_to(drone_map)

    # Save the map as an HTML file
    drone_map.save("drone_map.html")

# Function to open the map in a web browser
# def open_map():
#     chrome_driver_path = '/path/to/chromedriver'  # Update this with the path to ChromeDriver
#     service = Service(chrome_driver_path)
#     browser = webdriver.Chrome(service=service)
#     browser.get(f"file://{os.getcwd()}/drone_map.html")

# Connect to the vehicle (replace with your drone's connection string)
# vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to update drone location and map in real-time
# def track_drone():
#     while True:
#         # Get the current location of the drone
#         location = vehicle.location.global_frame
#         lat, lon = location.lat, location.lon
#         print(f"Current Drone Location -> Latitude: {lat}, Longitude: {lon}")

#         # Update the map with the current location
#         update_map(lat, lon)

#         # Update every 5 seconds
#         time.sleep(1)

# Function to send the drone to the input coordinates
def send_drone_to_location():
    try:
        lat = float(entry_latitude.get())
        lon = float(entry_longitude.get())
        alt = float(entry_altitude.get())

        # Create a LocationGlobalRelative object for the target location
        target_location = LocationGlobalRelative(lat, lon, alt)

        # Command the drone to fly to the target location
        print(f"Sending drone to Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
        vehicle.simple_goto(target_location)

    except ValueError:
        print("Invalid input. Please enter valid numbers for latitude, longitude, and altitude.")

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

# Latitude Label and Entry
label_latitude = tk.Label(window, text="Latitude:")
label_latitude.grid(row=0, column=0)
entry_latitude = tk.Entry(window)
entry_latitude.grid(row=0, column=1)

# Longitude Label and Entry
label_longitude = tk.Label(window, text="Longitude:")
label_longitude.grid(row=1, column=0)
entry_longitude = tk.Entry(window)
entry_longitude.grid(row=1, column=1)

# Altitude Label and Entry
label_altitude = tk.Label(window, text="Altitude:")
label_altitude.grid(row=2, column=0)
entry_altitude = tk.Entry(window)
entry_altitude.grid(row=2, column=1)

# Button to send the drone to the input location
send_button = tk.Button(window, text="Send Drone", command=send_drone_to_location)
send_button.grid(row=3, column=1)

# # Button to open the map
# map_button = tk.Button(window, text="Open Real-Time Map", command=open_map)
# map_button.grid(row=4, column=1)

# Button for LOITER mode
loiter_button = tk.Button(window, text="Loiter", command=set_loiter_mode)
loiter_button.grid(row=5, column=0)

# Button for RTL mode
rtl_button = tk.Button(window, text="RTL", command=set_rtl_mode)
rtl_button.grid(row=5, column=1)

# Start a separate thread to track the drone's location in real time
# tracking_thread = threading.Thread(target=track_drone)
# tracking_thread.start()

# Start the GUI loop
window.mainloop()

# Close the vehicle connection
vehicle.close()
