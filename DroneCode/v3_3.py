# import argparse
# import tkinter as tk
# from tkinter import messagebox
# from dronekit import connect, VehicleMode
# import time
# import threading
# from gpiozero import DistanceSensor
#
# # Parse command-line arguments for vehicle connection
# parser = argparse.ArgumentParser()
# parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
# args = parser.parse_args()
#
# # GPIO pins for HC-SR04 sensors (front, back, left, right)
# FRONT_TRIG, FRONT_ECHO = 17, 27
# LEFT_TRIG, LEFT_ECHO = 22, 23
# RIGHT_TRIG, RIGHT_ECHO = 5, 6
# BACK_TRIG, BACK_ECHO = 19, 26
#
# # Initialize distance sensors
# front_sensor = DistanceSensor(echo=FRONT_ECHO, trigger=FRONT_TRIG, max_distance=4)
# left_sensor = DistanceSensor(echo=LEFT_ECHO, trigger=LEFT_TRIG, max_distance=4)
# right_sensor = DistanceSensor(echo=RIGHT_ECHO, trigger=RIGHT_TRIG, max_distance=4)
# back_sensor = DistanceSensor(echo=BACK_ECHO, trigger=BACK_TRIG, max_distance=4)
#
# def measure_distance(sensor):
#     try:
#         return round(sensor.distance * 100, 2)  # Convert distance to centimeters
#     except Exception as e:
#         print(f"Error reading sensor: {e}")
#         return None
#
# class DroneControlGUI:
#     def __init__(self, master, connection_string):
#         self.master = master
#         try:
#             self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
#         except Exception as e:
#             messagebox.showerror("Error", f"Could not connect to drone: {e}")
#             master.destroy()
#             return
#
#         self.obstacle_prevent_enabled = False
#         self.obstacle_prevent_distance = 2  # Default obstacle avoidance distance (in meters)
#         self.latest_obstacle_status = "All Clear"  # Status of the latest obstacle detection
#
#         master.title("SAFE FLY")
#         master.geometry("800x600")
#
#         self.status_bar = tk.Label(master, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
#         self.status_bar.pack(side=tk.TOP, fill=tk.X)
#
#         self.create_widgets()
#         self.update_status()
#
#         self.obstacle_thread = threading.Thread(target=self.obstacle_avoidance_system)
#         self.obstacle_thread.daemon = True
#         self.obstacle_thread.start()
#
#     def create_widgets(self):
#         self.master.geometry("800x600")
#         self.master.resizable(True, True)
#
#         main_frame = tk.Frame(self.master)
#         main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
#
#         canvas = tk.Canvas(main_frame)
#         scrollbar = tk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
#         scrollable_frame = tk.Frame(canvas)
#
#         scrollable_frame.bind(
#             "<Configure>",
#             lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
#         )
#
#         canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
#         canvas.configure(yscrollcommand=scrollbar.set)
#
#         canvas.pack(side="left", fill="both", expand=True)
#         scrollbar.pack(side="right", fill="y")
#
#         control_frame = tk.Frame(scrollable_frame)
#         control_frame.pack(pady=10)
#
#         self.create_control_buttons(control_frame)
#         self.create_status_labels(scrollable_frame)
#
#     def create_control_buttons(self, control_frame):
#         arm_button = tk.Button(control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", width=10)
#         arm_button.grid(row=0, column=0, padx=5, pady=5)
#
#         disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=10)
#         disarm_button.grid(row=0, column=1, padx=5, pady=5)
#
#         rtl_button = tk.Button(control_frame, text="RTL (Return to Launch)", command=self.set_rtl_mode, bg="blue", fg="white", width=15)
#         rtl_button.grid(row=1, column=0, columnspan=2, pady=5)
#
#         loiter_button = tk.Button(control_frame, text="Loiter Mode", command=self.set_loiter_mode, bg="orange", fg="white", width=15)
#         loiter_button.grid(row=2, column=0, columnspan=2, pady=5)
#
#         land_button = tk.Button(control_frame, text="Land Mode", command=self.land_drone, bg="brown", fg="white", width=15)
#         land_button.grid(row=3, column=0, columnspan=2, pady=5)
#
#         takeoff_frame = tk.Frame(control_frame)
#         takeoff_frame.grid(row=4, column=0, columnspan=2, pady=20)
#
#         self.altitude_slider = tk.Scale(takeoff_frame, from_=0, to=30, orient=tk.HORIZONTAL, label="Takeoff Altitude (m)", length=200)
#         self.altitude_slider.set(0)
#         self.altitude_slider.pack(side=tk.LEFT, padx=10)
#
#         takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=10)
#         takeoff_button.pack(side=tk.LEFT)
#
#         self.obstacle_slider = tk.Scale(control_frame, from_=1, to=5, orient=tk.HORIZONTAL, label="Safe Distance (m)", length=200)
#         self.obstacle_slider.set(2)
#         self.obstacle_slider.grid(row=5, column=0, columnspan=2, pady=5)
#
#         self.avoidance_mode_button = tk.Button(control_frame, text="Enable Avoidance", command=self.toggle_avoidance_mode, bg="lightblue", width=15)
#         self.avoidance_mode_button.grid(row=6, column=0, columnspan=2, pady=5)
#
#     def create_status_labels(self, parent_frame):
#         status_frame = tk.Frame(parent_frame)
#         status_frame.pack(pady=10)
#
#         self.status_labels = {}
#         status_items = [
#             "Connection Status:", "Current Mode:", "Drone Location:",
#             "Altitude (m):", "Battery Level:", "Pitch:", "Roll:", "Yaw:", "Obstacle Status:"
#         ]
#
#         for i, item in enumerate(status_items):
#             label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
#             label.grid(row=i, column=0, sticky="w", padx=10)
#             value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
#             value.grid(row=i, column=1, sticky="w", padx=10)
#             self.status_labels[item] = value
#
#     def arm_drone(self):
#         if self.vehicle.is_armable:
#             self.vehicle.mode = VehicleMode("GUIDED")
#             self.vehicle.armed = True
#             while not self.vehicle.armed:
#                 time.sleep(1)
#             self.show_message("Info", "Drone armed!")
#             self.status_bar.config(text="Status: Drone Armed")
#         else:
#             self.show_message("Warning", "Drone is not armable. Check GPS and battery.")
#             self.status_bar.config(text="Status: Drone Not Armable")
#
#     def disarm_drone(self):
#         self.vehicle.armed = False
#         while self.vehicle.armed:
#             time.sleep(1)
#         self.show_message("Info", "Drone disarmed!")
#         self.status_bar.config(text="Status: Drone Disarmed")
#
#     def set_rtl_mode(self):
#         self.vehicle.mode = VehicleMode("RTL")
#         self.show_message("Info", "Switching to RTL (Return to Launch) mode.")
#         self.status_bar.config(text="Status: RTL Mode")
#
#     def set_loiter_mode(self):
#         self.vehicle.mode = VehicleMode("LOITER")
#         self.show_message("Info", "Switching to Loiter mode.")
#         self.status_bar.config(text="Status: Loiter Mode")
#
#     def land_drone(self):
#         self.vehicle.mode = VehicleMode("LAND")
#         self.show_message("Info", "Switching to Land mode.")
#         self.status_bar.config(text="Status: Landing")
#
#     def takeoff_drone(self):
#         altitude = self.altitude_slider.get()
#         self.vehicle.mode = VehicleMode("GUIDED")
#         self.vehicle.simple_takeoff(altitude)  # Take off to target altitude
#         self.show_message("Info", f"Taking off to {altitude} meters.")
#
#     def toggle_avoidance_mode(self):
#         self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
#         self.avoidance_mode_button.config(text="Disable Avoidance" if self.obstacle_prevent_enabled else "Enable Avoidance")
#         self.show_message("Info", "Obstacle avoidance " + ("enabled." if self.obstacle_prevent_enabled else "disabled."))
#
#     def obstacle_avoidance_system(self):
#         while True:
#             try:
#                 if self.obstacle_prevent_enabled:
#                     self.obstacle_prevent_distance = self.obstacle_slider.get()
#                     front_distance = measure_distance(front_sensor)
#                     back_distance = measure_distance(back_sensor)
#                     left_distance = measure_distance(left_sensor)
#                     right_distance = measure_distance(right_sensor)
#
#                     obstacle_status = "All Clear"
#                     if front_distance < self.obstacle_prevent_distance:
#                         obstacle_status = "Front Obstacle Detected"
#                     elif back_distance < self.obstacle_prevent_distance:
#                         obstacle_status = "Back Obstacle Detected"
#                     elif left_distance < self.obstacle_prevent_distance:
#                         obstacle_status = "Left Obstacle Detected"
#                     elif right_distance < self.obstacle_prevent_distance:
#                         obstacle_status = "Right Obstacle Detected"
#
#                     self.latest_obstacle_status = obstacle_status
#                     self.status_labels["Obstacle Status:"].config(text=self.latest_obstacle_status)
#
#                 time.sleep(1)
#             except Exception as e:
#                 print(f"Error in obstacle avoidance system: {e}")
#
#     def update_status(self):
#         if self.vehicle:
#             self.status_labels["Connection Status:"].config(text="Connected")
#             self.status_labels["Current Mode:"].config(text=str(self.vehicle.mode.name))
#             self.status_labels["Drone Location:"].config(text=str(self.vehicle.location.global_frame))
#             self.status_labels["Altitude (m):"].config(text=f"{self.vehicle.location.global_relative_frame.alt:.2f}")
#             self.status_labels["Battery Level:"].config(text=str(self.vehicle.battery))
#             self.status_labels["Pitch:"].config(text=f"{self.vehicle.attitude.pitch:.2f}")
#             self.status_labels["Roll:"].config(text=f"{self.vehicle.attitude.roll:.2f}")
#             self.status_labels["Yaw:"].config(text=f"{self.vehicle.attitude.yaw:.2f}")
#
#         self.master.after(1000, self.update_status)
#
#     def show_message(self, title, message):
#         messagebox.showinfo(title, message)
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     DroneControlGUI(root, args.connect)
#     root.mainloop()
#
#
#--------------------------------------------------------------------- 


# import argparse
# import tkinter as tk
# from tkinter import messagebox
# from dronekit import connect, VehicleMode
# import time
# from gpiozero import DistanceSensor
#
# # Parse command-line arguments for vehicle connection
# parser = argparse.ArgumentParser()
# parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
# args = parser.parse_args()
#
# # GPIO pins for HC-SR04 sensors (front, back, left, right)
# FRONT_TRIG, FRONT_ECHO = 17, 27
# LEFT_TRIG, LEFT_ECHO = 22, 23
# RIGHT_TRIG, RIGHT_ECHO = 5, 6
# BACK_TRIG, BACK_ECHO = 19, 26
#
# # Initialize distance sensors
# front_sensor = DistanceSensor(echo=FRONT_ECHO, trigger=FRONT_TRIG, max_distance=4)
# left_sensor = DistanceSensor(echo=LEFT_ECHO, trigger=LEFT_TRIG, max_distance=4)
# right_sensor = DistanceSensor(echo=RIGHT_ECHO, trigger=RIGHT_TRIG, max_distance=4)
# back_sensor = DistanceSensor(echo=BACK_ECHO, trigger=BACK_TRIG, max_distance=4)
#
#
# def measure_distance(sensor):
#     try:
#         return round(sensor.distance * 100, 2)  # Convert distance to centimeters
#     except Exception as e:
#         print(f"Error reading sensor: {e}")
#         return None
#
#
# class DroneControlGUI:
#     def __init__(self, master, connection_string):
#         self.master = master
#
#         try:
#             self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
#         except Exception as e:
#             messagebox.showerror("Error", f"Could not connect to drone: {e}")
#             master.destroy()
#             return
#
#         self.obstacle_prevent_enabled = False
#         self.obstacle_prevent_distance = 2  # Default obstacle avoidance distance (in meters)
#         self.latest_obstacle_status = "All Clear"  # Status of the latest obstacle detection
#
#         master.title("SAFE FLY")
#         master.geometry("800x600")
#
#         self.status_bar = tk.Label(master, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
#         self.status_bar.pack(side=tk.TOP, fill=tk.X)
#
#         self.create_widgets()
#         self.update_status()  # Start updating GUI and obstacle detection in the same loop
#
#     def create_widgets(self):
#         main_frame = tk.Frame(self.master)
#         main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
#
#         control_frame = tk.Frame(main_frame)
#         control_frame.pack(pady=10)
#
#         self.create_control_buttons(control_frame)
#         self.create_status_labels(main_frame)
#
#     def create_control_buttons(self, control_frame):
#         arm_button = tk.Button(control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", width=10)
#         arm_button.grid(row=0, column=0, padx=5, pady=5)
#
#         disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=10)
#         disarm_button.grid(row=0, column=1, padx=5, pady=5)
#
#         rtl_button = tk.Button(control_frame, text="RTL (Return to Launch)", command=self.set_rtl_mode, bg="blue", fg="white", width=15)
#         rtl_button.grid(row=1, column=0, columnspan=2, pady=5)
#
#         loiter_button = tk.Button(control_frame, text="Loiter Mode", command=self.set_loiter_mode, bg="orange", fg="white", width=15)
#         loiter_button.grid(row=2, column=0, columnspan=2, pady=5)
#
#         land_button = tk.Button(control_frame, text="Land Mode", command=self.land_drone, bg="brown", fg="white", width=15)
#         land_button.grid(row=3, column=0, columnspan=2, pady=5)
#
#         takeoff_frame = tk.Frame(control_frame)
#         takeoff_frame.grid(row=4, column=0, columnspan=2, pady=20)
#
#         self.altitude_slider = tk.Scale(takeoff_frame, from_=0, to=30, orient=tk.HORIZONTAL, label="Takeoff Altitude (m)", length=200)
#         self.altitude_slider.set(0)
#         self.altitude_slider.pack(side=tk.LEFT, padx=10)
#
#         takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=10)
#         takeoff_button.pack(side=tk.LEFT)
#
#         self.obstacle_slider = tk.Scale(control_frame, from_=1, to=5, orient=tk.HORIZONTAL, label="Safe Distance (m)", length=200)
#         self.obstacle_slider.set(2)
#         self.obstacle_slider.grid(row=5, column=0, columnspan=2, pady=5)
#
#         self.avoidance_mode_button = tk.Button(control_frame, text="Enable Avoidance", command=self.toggle_avoidance_mode, bg="lightblue", width=15)
#         self.avoidance_mode_button.grid(row=6, column=0, columnspan=2, pady=5)
#
#     def create_status_labels(self, parent_frame):
#         status_frame = tk.Frame(parent_frame)
#         status_frame.pack(pady=10)
#
#         self.status_labels = {}
#         status_items = [
#             "Connection Status:", "Current Mode:", "Drone Location:",
#             "Altitude (m):", "Battery Level:", "Pitch:", "Roll:", "Yaw:", "Obstacle Status:"
#         ]
#
#         for i, item in enumerate(status_items):
#             label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
#             label.grid(row=i, column=0, sticky="w", padx=10)
#             value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
#             value.grid(row=i, column=1, sticky="w", padx=10)
#             self.status_labels[item] = value
#
#     def arm_drone(self):
#         if self.vehicle.is_armable:
#             self.vehicle.mode = VehicleMode("GUIDED")
#             self.vehicle.armed = True
#             while not self.vehicle.armed:
#                 time.sleep(1)
#             self.show_message("Info", "Drone armed!")
#             self.status_bar.config(text="Status: Drone Armed")
#         else:
#             self.show_message("Warning", "Drone is not armable. Check GPS and battery.")
#             self.status_bar.config(text="Status: Drone Not Armable")
#
#     def disarm_drone(self):
#         self.vehicle.armed = False
#         while self.vehicle.armed:
#             time.sleep(1)
#         self.show_message("Info", "Drone disarmed!")
#         self.status_bar.config(text="Status: Drone Disarmed")
#
#     def set_rtl_mode(self):
#         self.vehicle.mode = VehicleMode("RTL")
#         self.show_message("Info", "Switching to RTL (Return to Launch) mode.")
#         self.status_bar.config(text="Status: RTL Mode")
#
#     def set_loiter_mode(self):
#         self.vehicle.mode = VehicleMode("LOITER")
#         self.show_message("Info", "Switching to Loiter mode.")
#         self.status_bar.config(text="Status: Loiter Mode")
#
#     def land_drone(self):
#         self.vehicle.mode = VehicleMode("LAND")
#         self.show_message("Info", "Switching to Land mode.")
#         self.status_bar.config(text="Status: Landing")
#
#     def takeoff_drone(self):
#         altitude = self.altitude_slider.get()
#         self.vehicle.mode = VehicleMode("GUIDED")
#         self.vehicle.simple_takeoff(altitude)  # Take off to target altitude
#         self.show_message("Info", f"Taking off to {altitude} meters.")
#
#     def toggle_avoidance_mode(self):
#         self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
#         self.avoidance_mode_button.config(text="Disable Avoidance" if self.obstacle_prevent_enabled else "Enable Avoidance")
#         self.show_message("Info", "Obstacle avoidance " + ("enabled." if self.obstacle_prevent_enabled else "disabled."))
#
#     def update_status(self):
#         if self.vehicle:
#             self.status_labels["Connection Status:"].config(text="Connected")
#             self.status_labels["Current Mode:"].config(text=str(self.vehicle.mode.name))
#             self.status_labels["Drone Location:"].config(text=str(self.vehicle.location.global_frame))
#             self.status_labels["Altitude (m):"].config(text=f"{self.vehicle.location.global_relative_frame.alt:.2f}")
#             self.status_labels["Battery Level:"].config(text=str(self.vehicle.battery))
#             self.status_labels["Pitch:"].config(text=f"{self.vehicle.attitude.pitch:.2f}")
#             self.status_labels["Roll:"].config(text=f"{self.vehicle.attitude.roll:.2f}")
#             self.status_labels["Yaw:"].config(text=f"{self.vehicle.attitude.yaw:.2f}")
#
#         # Obstacle detection in the same main loop
#         if self.obstacle_prevent_enabled:
#             self.obstacle_prevent_distance = self.obstacle_slider.get()
#             front_distance = measure_distance(front_sensor)
#             back_distance = measure_distance(back_sensor)
#             left_distance = measure_distance(left_sensor)
#             right_distance = measure_distance(right_sensor)
#
#             obstacle_status = "All Clear"
#             if front_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Front Obstacle Detected"
#             elif back_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Back Obstacle Detected"
#             elif left_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Left Obstacle Detected"
#             elif right_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Right Obstacle Detected"
#
#             self.latest_obstacle_status = obstacle_status
#             self.status_labels["Obstacle Status:"].config(text=self.latest_obstacle_status)
#
#         self.master.after(1000, self.update_status)  # Continuously update every second
#
#     def show_message(self, title, message):
#         messagebox.showinfo(title, message)
#
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     DroneControlGUI(root, args.connect)
#     root.mainloop()

#-------------------------------------------------------------------------

# import argparse
# import tkinter as tk
# from tkinter import messagebox
# from dronekit import connect, VehicleMode
# import time
# from gpiozero import DistanceSensor
#
# # Parse command-line arguments for vehicle connection
# parser = argparse.ArgumentParser()
# parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
# args = parser.parse_args()
#
# # GPIO pins for HC-SR04 sensors (front, back, left, right)
# FRONT_TRIG, FRONT_ECHO = 17, 27
# LEFT_TRIG, LEFT_ECHO = 22, 23
# RIGHT_TRIG, RIGHT_ECHO = 5, 6
# BACK_TRIG, BACK_ECHO = 19, 26
#
# # Initialize distance sensors
# front_sensor = DistanceSensor(echo=FRONT_ECHO, trigger=FRONT_TRIG, max_distance=4)
# left_sensor = DistanceSensor(echo=LEFT_ECHO, trigger=LEFT_TRIG, max_distance=4)
# right_sensor = DistanceSensor(echo=RIGHT_ECHO, trigger=RIGHT_TRIG, max_distance=4)
# back_sensor = DistanceSensor(echo=BACK_ECHO, trigger=BACK_TRIG, max_distance=4)
#
#
# def measure_distance(sensor):
#     try:
#         return round(sensor.distance * 100, 2)  # Convert distance to centimeters
#     except Exception as e:
#         print(f"Error reading sensor: {e}")
#         return None
#
#
# class DroneControlGUI:
#     def __init__(self, master, connection_string):
#         self.master = master
#
#         try:
#             self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
#         except Exception as e:
#             messagebox.showerror("Error", f"Could not connect to drone: {e}")
#             master.destroy()
#             return
#
#         self.obstacle_prevent_enabled = False
#         self.obstacle_prevent_distance = 2  # Default obstacle avoidance distance (in meters)
#         # self.latest_obstacle_status = "All Clear"  # Status of the latest obstacle detection
#
#         master.title("SAFE FLY")
#         master.geometry("800x600")
#
#         # Create a Canvas widget for scrolling
#         canvas = tk.Canvas(master)
#         canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
#
#         # Create a Scrollbar and link it to the canvas
#         scrollbar = tk.Scrollbar(master, orient="vertical", command=canvas.yview)
#         scrollbar.pack(side=tk.RIGHT, fill="y")
#
#         canvas.configure(yscrollcommand=scrollbar.set)
#
#         # Create a Frame to hold all the widgets
#         self.main_frame = tk.Frame(canvas)
#         canvas.create_window((0, 0), window=self.main_frame, anchor="nw")
#
#         self.main_frame.bind(
#             "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
#         )
#
#         self.status_bar = tk.Label(master, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
#         self.status_bar.pack(side=tk.TOP, fill=tk.X)
#
#         self.create_widgets()
#         self.update_status()  # Start updating GUI and obstacle detection in the same loop
#
#     def create_widgets(self):
#         control_frame = tk.Frame(self.main_frame)
#         control_frame.pack(pady=10)
#
#         self.create_control_buttons(control_frame)
#         self.create_status_labels(self.main_frame)
#
#     def create_control_buttons(self, control_frame):
#         arm_button = tk.Button(control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", width=10)
#         arm_button.grid(row=0, column=0, padx=5, pady=5)
#
#         disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=10)
#         disarm_button.grid(row=0, column=1, padx=5, pady=5)
#
#         rtl_button = tk.Button(control_frame, text="RTL (Return to Launch)", command=self.set_rtl_mode, bg="blue", fg="white", width=15)
#         rtl_button.grid(row=1, column=0, columnspan=2, pady=5)
#
#         loiter_button = tk.Button(control_frame, text="Loiter Mode", command=self.set_loiter_mode, bg="orange", fg="white", width=15)
#         loiter_button.grid(row=2, column=0, columnspan=2, pady=5)
#
#         land_button = tk.Button(control_frame, text="Land Mode", command=self.land_drone, bg="brown", fg="white", width=15)
#         land_button.grid(row=3, column=0, columnspan=2, pady=5)
#
#         takeoff_frame = tk.Frame(control_frame)
#         takeoff_frame.grid(row=4, column=0, columnspan=2, pady=20)
#
#         self.altitude_slider = tk.Scale(takeoff_frame, from_=0, to=30, orient=tk.HORIZONTAL, label="Takeoff Altitude (m)", length=200)
#         self.altitude_slider.set(0)
#         self.altitude_slider.pack(side=tk.LEFT, padx=10)
#
#         takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=10)
#         takeoff_button.pack(side=tk.LEFT)
#
#         self.obstacle_slider = tk.Scale(control_frame, from_=1, to=5, orient=tk.HORIZONTAL, label="Safe Distance (m)", length=200)
#         self.obstacle_slider.set(2)
#         self.obstacle_slider.grid(row=5, column=0, columnspan=2, pady=5)
#
#         self.avoidance_mode_button = tk.Button(control_frame, text="Enable Avoidance", command=self.toggle_avoidance_mode, bg="lightblue", width=15)
#         self.avoidance_mode_button.grid(row=6, column=0, columnspan=2, pady=5)
#
#     def create_status_labels(self, parent_frame):
#         status_frame = tk.Frame(parent_frame)
#         status_frame.pack(pady=10)
#
#         self.status_labels = {}
#         status_items = [
#             "Connection Status:", "Current Mode:", "Drone Location:",
#             "Altitude (m):", "Battery Level:", "Pitch:", "Roll:", "Yaw:", "Obstacle Status:"
#         ]
#
#         for i, item in enumerate(status_items):
#             label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
#             label.grid(row=i, column=0, sticky="w", padx=10)
#             value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
#             value.grid(row=i, column=1, sticky="w", padx=10)
#             self.status_labels[item] = value
#
#     def arm_drone(self):
#         if self.vehicle.is_armable:
#             self.vehicle.mode = VehicleMode("GUIDED")
#             self.vehicle.armed = True
#             while not self.vehicle.armed:
#                 time.sleep(1)
#             self.show_message("Info", "Drone armed!")
#             self.status_bar.config(text="Status: Drone Armed")
#         else:
#             self.show_message("Warning", "Drone is not armable. Check GPS and battery.")
#             self.status_bar.config(text="Status: Drone Not Armable")
#
#     def disarm_drone(self):
#         self.vehicle.armed = False
#         while self.vehicle.armed:
#             time.sleep(1)
#         self.show_message("Info", "Drone disarmed!")
#         self.status_bar.config(text="Status: Drone Disarmed")
#
#     def set_rtl_mode(self):
#         self.vehicle.mode = VehicleMode("RTL")
#         self.show_message("Info", "Switching to RTL (Return to Launch) mode.")
#         self.status_bar.config(text="Status: RTL Mode")
#
#     def set_loiter_mode(self):
#         self.vehicle.mode = VehicleMode("LOITER")
#         self.show_message("Info", "Switching to Loiter mode.")
#         self.status_bar.config(text="Status: Loiter Mode")
#
#     def land_drone(self):
#         self.vehicle.mode = VehicleMode("LAND")
#         self.show_message("Info", "Switching to Land mode.")
#         self.status_bar.config(text="Status: Landing")
#
#     def takeoff_drone(self):
#         altitude = self.altitude_slider.get()
#         self.vehicle.mode = VehicleMode("GUIDED")
#         self.vehicle.simple_takeoff(altitude)  # Take off to target altitude
#         self.show_message("Info", f"Taking off to {altitude} meters.")
#
#     def toggle_avoidance_mode(self):
#         self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
#         self.avoidance_mode_button.config(text="Disable Avoidance" if self.obstacle_prevent_enabled else "Enable Avoidance")
#         self.show_message("Info", "Obstacle avoidance " + ("enabled." if self.obstacle_prevent_enabled else "disabled."))
#
#     def update_status(self):
#         if self.vehicle:
#             self.status_labels["Connection Status:"].config(text="Connected")
#             self.status_labels["Current Mode:"].config(text=str(self.vehicle.mode.name))
#             self.status_labels["Drone Location:"].config(text=str(self.vehicle.location.global_frame))
#             self.status_labels["Altitude (m):"].config(text=f"{self.vehicle.location.global_relative_frame.alt:.2f}")
#             self.status_labels["Battery Level:"].config(text=str(self.vehicle.battery))
#             self.status_labels["Pitch:"].config(text=f"{self.vehicle.attitude.pitch:.2f}")
#             self.status_labels["Roll:"].config(text=f"{self.vehicle.attitude.roll:.2f}")
#             self.status_labels["Yaw:"].config(text=f"{self.vehicle.attitude.yaw:.2f}")
#
#             front_distance = measure_distance(front_sensor)
#             back_distance = measure_distance(back_sensor)
#             left_distance = measure_distance(left_sensor)
#             right_distance = measure_distance(right_sensor)
#
#             obstacle_status = "All Clear"
#             if front_distance and front_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Front Obstacle Detected"
#             elif back_distance and back_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Back Obstacle Detected"
#             elif left_distance and left_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Left Obstacle Detected"
#             elif right_distance and right_distance < self.obstacle_prevent_distance:
#                 obstacle_status = "Right Obstacle Detected"
#
#             self.latest_obstacle_status = obstacle_status
#             self.status_labels["Obstacle Status:"].config(text=self.latest_obstacle_status)
#
#         self.master.after(1000, self.update_status)  # Continuously update every second
#
#     def show_message(self, title, message):
#         messagebox.showinfo(title, message)
#
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     DroneControlGUI(root, args.connect)
#     root.mainloop()


#----------------------------------------

# import tkinter as tk
# from gpiozero import DistanceSensor
# import time
# from threading import Thread

# # GPIO pins for HC-SR04 sensors (front, back, left, right)
# SENSORS_CONFIG = {
#     "Front": {"trigger": 17, "echo": 27},
#     "Left": {"trigger": 22, "echo": 23},
#     "Right": {"trigger": 5, "echo": 6},
#     "Back": {"trigger": 19, "echo": 26},
# }

# # Initialize sensors
# sonar_sensors = {
#     name: DistanceSensor(echo=pin_config["echo"], trigger=pin_config["trigger"], max_distance=4)
#     for name, pin_config in SENSORS_CONFIG.items()
# }

# class SonarDistanceApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Sonar Distance Measurement (Front, Back, Left, Right Sensors)")

#         self.distance_labels = {}
#         self.alert_messages = {}

#         for name in SENSORS_CONFIG.keys():
#             label = tk.Label(master, text=f"{name} Sensor Distance: N/A", font=('Arial', 14))
#             label.pack(pady=10)
#             self.distance_labels[name] = label

#             alert_label = tk.Label(master, text="", font=('Arial', 12), fg="red")
#             alert_label.pack()
#             self.alert_messages[name] = alert_label

#         # Start a background thread to measure distances
#         self.stop_thread = False
#         self.measure_thread = Thread(target=self.measure_distance_loop)
#         self.measure_thread.start()

#         # Add a quit button
#         quit_button = tk.Button(master, text="Quit", command=self.close_app)
#         quit_button.pack(pady=10)

#     def measure_distance_loop(self):
#         while not self.stop_thread:
#             try:
#                 # Measure distance for each sensor
#                 distances = {
#                     name: round(sensor.distance * 100, 2)  # Convert to centimeters
#                     for name, sensor in sonar_sensors.items()
#                 }
#                 self.update_distances(distances)
#                 time.sleep(1)  # Adjust delay as needed
#             except Exception as e:
#                 # Update GUI with an error message
#                 for name in self.distance_labels.keys():
#                     self.distance_labels[name].config(text=f"{name} Sensor Distance: Error")
#                     self.alert_messages[name].config(text="Measurement Error")

#     def update_distances(self, distances):
#         for name, distance in distances.items():
#             self.distance_labels[name].config(text=f"{name} Sensor Distance: {distance} cm")
#             if distance < 20:  # Example alert threshold for objects within 20 cm
#                 self.alert_messages[name].config(text=f"Alert: Object too close at {name} Sensor!")
#             else:
#                 self.alert_messages[name].config(text="")

#     def close_app(self):
#         self.stop_thread = True
#         self.measure_thread.join()
#         self.master.destroy()

# if __name__ == "__main__":
#     root = tk.Tk()
#     app = SonarDistanceApp(root)
#     root.mainloop()



#-----------------------------------------------


import argparse
import tkinter as tk
from tkinter import messagebox
from dronekit import connect, VehicleMode
import time
import threading
from gpiozero import DistanceSensor
# import RPi.GPIO as GPIO

# Add argparse to parse command-line arguments for vehicle connection
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
args = parser.parse_args()

# GPIO setup for HC-SR04 on multiple directions (front, back, left, right)
FRONT_TRIG, FRONT_ECHO = 17, 27
LEFT_TRIG, LEFT_ECHO = 22, 23
RIGHT_TRIG, RIGHT_ECHO = 5, 6
BACK_TRIG, BACK_ECHO = 19, 26

# Initialize DistanceSensor objects for each sensor
front_sensor = DistanceSensor(echo=FRONT_ECHO, trigger=FRONT_TRIG, max_distance=4)
left_sensor = DistanceSensor(echo=LEFT_ECHO, trigger=LEFT_TRIG, max_distance=4)
right_sensor = DistanceSensor(echo=RIGHT_ECHO, trigger=RIGHT_TRIG, max_distance=4)
back_sensor = DistanceSensor(echo=BACK_ECHO, trigger=BACK_TRIG, max_distance=4)

def measure_distance(sensor):
    try:
        return round(sensor.distance * 100, 2)  # Convert to centimeters
    except Exception as e:
        print(f"Error reading sensor: {e}")
        return None

class DroneControlGUI:
   
    def __init__(self, master, connection_string):
        self.master = master
        self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
        self.obstacle_prevent_enabled = False
        self.obstacle_prevent_distance = 2  # Default obstacle avoidance distance in meters
        self.latest_obstacle_status = "All Clear"  # Store the latest status
        
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
        # Configure root window for resizing
        self.master.geometry("800x600")
        self.master.resizable(True, True)

        # Main frame with scrollable canvas
        main_frame = tk.Frame(self.master)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Scrollable canvas
        canvas = tk.Canvas(main_frame)
        scrollbar = tk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Pack the canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Control Buttons Frame
        control_frame = tk.Frame(scrollable_frame)
        control_frame.pack(pady=10)

        # Control Buttons
        arm_button = tk.Button(control_frame, text="ARM", command=self.arm_drone, bg="green", fg="white", width=10)
        arm_button.grid(row=0, column=0, padx=5, pady=5)

        disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_drone, bg="red", fg="white", width=10)
        disarm_button.grid(row=0, column=1, padx=5, pady=5)

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
        self.altitude_slider.set(0)
        self.altitude_slider.pack(side=tk.LEFT, padx=10)

        takeoff_button = tk.Button(takeoff_frame, text="Takeoff", command=self.takeoff_drone, bg="cyan", fg="black", width=10)
        takeoff_button.pack(side=tk.LEFT)

        # Obstacle Avoidance Slider
        self.obstacle_slider = tk.Scale(control_frame, from_=1, to=5, orient=tk.HORIZONTAL, label="Safe Distance (m)", length=200)
        self.obstacle_slider.set(2)
        self.obstacle_slider.grid(row=5, column=0, columnspan=2, pady=5)

        # Enable/Disable Obstacle Avoidance
        self.avoidance_mode_button = tk.Button(control_frame, text="Enable Avoidance", command=self.toggle_avoidance_mode, bg="lightblue", width=15)
        self.avoidance_mode_button.grid(row=6, column=0, columnspan=2, pady=5)

        stabilize_button = tk.Button(control_frame, text="Stabilize Mode", command=self.set_stabilize_mode, bg="purple", fg="white", width=15)
        stabilize_button.grid(row=7, column=0, columnspan=2, pady=5)

        smart_rtl_button = tk.Button(control_frame, text="Smart RTL", command=self.set_smart_rtl_mode, bg="yellow", fg="black", width=15)
        smart_rtl_button.grid(row=8, column=0, columnspan=2, pady=5)

        athold_button = tk.Button(control_frame, text="ATHOLD Mode", command=self.set_athold_mode, bg="purple", fg="white", width=15)
        athold_button.grid(row=9, column=0, columnspan=2, pady=5)

        break_button = tk.Button(control_frame, text="BREAK Mode", command=self.set_break_mode, bg="gray", fg="white", width=15)
        break_button.grid(row=10, column=0, columnspan=2, pady=5)

        # Status Display
        status_frame = tk.Frame(scrollable_frame)
        status_frame.pack(pady=10)

        self.status_labels = {}
        status_items = [
            "Connection Status:", "Current Mode:", "Drone Location:",
            "Altitude (m):", "Battery Level:", "Pitch:", "Roll:", "Yaw:", "Obstacle Status:"
        ]

        for i, item in enumerate(status_items):
            label = tk.Label(status_frame, text=item, font=('Arial', 10, 'bold'))
            label.grid(row=i, column=0, sticky="w", padx=10)
            value = tk.Label(status_frame, text="N/A", font=('Arial', 10))
            value.grid(row=i, column=1, sticky="w", padx=10)
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
        self.status_bar.config(text="Status: Land Mode")

    def takeoff_drone(self):
        altitude = self.altitude_slider.get()
        self.vehicle.simple_takeoff(altitude)
        while self.vehicle.location.global_relative_frame.alt < altitude * 0.95:
            time.sleep(1)
        self.show_message("Info", "Takeoff complete!")
        self.status_bar.config(text="Status: Takeoff Complete")

    def set_stabilize_mode(self):
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.show_message("Info", "Switching to Stabilize mode.")
        self.status_bar.config(text="Status: Stabilize Mode")

    def set_smart_rtl_mode(self):
        self.vehicle.mode = VehicleMode("RTL")
        self.show_message("Info", "Switching to Smart RTL mode.")
        self.status_bar.config(text="Status: Smart RTL Mode")

    def set_athold_mode(self):
        self.vehicle.mode = VehicleMode("ATHOLD")
        self.show_message("Info", "Switching to ATHOLD mode.")
        self.status_bar.config(text="Status: ATHOLD Mode")

    def set_break_mode(self):
        self.vehicle.mode = VehicleMode("BREAK")
        self.show_message("Info", "Switching to BREAK mode.")
        self.status_bar.config(text="Status: BREAK Mode")

    def show_message(self, title, message):
        messagebox.showinfo(title, message)

    def toggle_avoidance_mode(self):
        self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
        if self.obstacle_prevent_enabled:
            self.avoidance_mode_button.config(text="Disable Avoidance")
        else:
            self.avoidance_mode_button.config(text="Enable Avoidance")
        self.status_bar.config(text="Status: Obstacle Avoidance " + ("Enabled" if self.obstacle_prevent_enabled else "Disabled"))

    def update_status(self):
        try:
            self.status_labels["Connection Status:"].config(text="Connected")
            self.status_labels["Current Mode:"].config(text=self.vehicle.mode.name)
            self.status_labels["Drone Location:"].config(text=f"{self.vehicle.location.global_frame.lat}, {self.vehicle.location.global_frame.lon}")
            self.status_labels["Altitude (m):"].config(text=str(round(self.vehicle.location.global_relative_frame.alt, 2)))
            self.status_labels["Battery Level:"].config(text=f"{self.vehicle.battery.level}%")
            self.status_labels["Pitch:"].config(text=str(self.vehicle.attitude.pitch))
            self.status_labels["Roll:"].config(text=str(self.vehicle.attitude.roll))
            self.status_labels["Yaw:"].config(text=str(self.vehicle.attitude.yaw))
            self.status_labels["Obstacle Status:"].config(text=self.latest_obstacle_status)
        except Exception as e:
            print(f"Error updating status: {e}")
        self.master.after(1000, self.update_status)

    def obstacle_avoidance_system(self):
        while True:
            if self.obstacle_prevent_enabled:
                front_distance = measure_distance(front_sensor)
                left_distance = measure_distance(left_sensor)
                right_distance = measure_distance(right_sensor)
                back_distance = measure_distance(back_sensor)

                obstacle_detected = False
                if front_distance is not None and front_distance < self.obstacle_slider.get():
                    self.latest_obstacle_status = "Obstacle in front!"
                    obstacle_detected = True
                elif left_distance is not None and left_distance < self.obstacle_slider.get():
                    self.latest_obstacle_status = "Obstacle on left!"
                    obstacle_detected = True
                elif right_distance is not None and right_distance < self.obstacle_slider.get():
                    self.latest_obstacle_status = "Obstacle on right!"
                    obstacle_detected = True
                elif back_distance is not None and back_distance < self.obstacle_slider.get():
                    self.latest_obstacle_status = "Obstacle behind!"
                    obstacle_detected = True
                else:
                    self.latest_obstacle_status = "All Clear"

                if obstacle_detected:
                    self.vehicle.mode = VehicleMode("HOLD")
                else:
                    if self.vehicle.mode != VehicleMode("GUIDED"):
                        self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.5)

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneControlGUI(root, args.connect)
    root.mainloop()
