# import argparse
# import time
# import threading
# from dronekit import connect, VehicleMode
# from gpiozero import DistanceSensor

# # Add argparse to parse command-line arguments for vehicle connection
# parser = argparse.ArgumentParser()
# parser.add_argument('--connect', default='127.0.0.1:14550', help='Connection string to the vehicle')
# args = parser.parse_args()

# # GPIO setup for HC-SR04 on multiple directions (front, back, left, right)
# FRONT_TRIG, FRONT_ECHO = 17, 27
# LEFT_TRIG, LEFT_ECHO = 22, 23
# RIGHT_TRIG, RIGHT_ECHO = 5, 6
# BACK_TRIG, BACK_ECHO = 19, 26

# # Initialize DistanceSensor objects for each sensor
# front_sensor = DistanceSensor(echo=FRONT_ECHO, trigger=FRONT_TRIG, max_distance=4)
# left_sensor = DistanceSensor(echo=LEFT_ECHO, trigger=LEFT_TRIG, max_distance=4)
# right_sensor = DistanceSensor(echo=RIGHT_ECHO, trigger=RIGHT_TRIG, max_distance=4)
# back_sensor = DistanceSensor(echo=BACK_ECHO, trigger=BACK_TRIG, max_distance=4)

# def measure_distance(sensor):
#     try:
#         distance = round(sensor.distance * 100, 2)  # Convert to centimeters
#         if distance is None or distance > 400:  # No object detected or sensor out of range
#             return 999  # Default large value indicating no obstacle
#         return distance
#     except Exception as e:
#         print(f"Error reading sensor: {e}")
#         return 999  # Default value when error occurs

# class DroneObstacleAvoidance:

#     def __init__(self, connection_string):
#         # Connect to the vehicle
#         self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
#         self.obstacle_prevent_enabled = False
#         self.obstacle_prevent_distance = 200  # Default obstacle avoidance distance in cm

#         # Start the obstacle avoidance system
#         self.obstacle_thread = threading.Thread(target=self.obstacle_avoidance_system)
#         self.obstacle_thread.daemon = True
#         self.obstacle_thread.start()

#     def toggle_avoidance_mode(self):
#         self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
#         print(f"Obstacle avoidance {'enabled' if self.obstacle_prevent_enabled else 'disabled'}.")

#     def obstacle_avoidance_system(self):
#         while True:
#             try:
#                 if self.obstacle_prevent_enabled:
#                     # Measure distances from sensors
#                     front_distance = measure_distance(front_sensor)
#                     back_distance = measure_distance(back_sensor)
#                     left_distance = measure_distance(left_sensor)
#                     right_distance = measure_distance(right_sensor)

#                     print(f"Front: {front_distance} cm, Back: {back_distance} cm, Left: {left_distance} cm, Right: {right_distance} cm")

#                     # Ensure the drone is in GUIDED mode
#                     if self.vehicle.mode != VehicleMode("GUIDED"):
#                         print("Switching to GUIDED mode")
#                         self.vehicle.mode = VehicleMode("GUIDED")

#                     # Obstacle detection flags
#                     obstacle_front = front_distance < self.obstacle_prevent_distance
#                     obstacle_back = back_distance < self.obstacle_prevent_distance
#                     obstacle_left = left_distance < self.obstacle_prevent_distance
#                     obstacle_right = right_distance < self.obstacle_prevent_distance

#                     # Decision Logic for Obstacle Avoidance
#                     if (obstacle_front and obstacle_back) or (obstacle_left and obstacle_right):
#                         self.vehicle.simple_goto(self.vehicle.location.global_frame.lat,
#                                                  self.vehicle.location.global_frame.lon,
#                                                  self.vehicle.location.global_frame.alt + 1)
#                         print("Obstacles in multiple directions - Increasing Height!")
#                     else:
#                         # Handle obstacles in individual directions
#                         # Additional logic can be added here as needed

#                     time.sleep(0.5)  # Wait half a second before the next check
#             except Exception as e:
#                 print(f"Error measuring distance: {e}")
#                 break

#     def close(self):
#         self.vehicle.close()

# # Main code execution
# if __name__ == '__main__':
#     drone_control = DroneObstacleAvoidance(args.connect)
#     try:
#         while True:
#             # Example of enabling and disabling obstacle avoidance mode
#             user_input = input("Enter 'enable' to activate obstacle avoidance, 'disable' to deactivate: ").strip().lower()
#             if user_input == 'enable':
#                 drone_control.toggle_avoidance_mode()
#             elif user_input == 'disable':
#                 drone_control.toggle_avoidance_mode()
#             else:
#                 print("Invalid input. Please type 'enable' or 'disable'.")
#             time.sleep(1)

#     except KeyboardInterrupt:
#         print("Exiting...")
#         drone_control.close()

import argparse
import time
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative,mavutil
from gpiozero import DistanceSensor

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
        distance = round(sensor.distance * 100, 2)  # Convert to centimeters
        if distance > 400:  # No object detected or sensor out of range
            return 999  # Default large value indicating no obstacle
        return distance
    except Exception as e:
        print(f"Error reading sensor: {e}")
        return 999  # Default value when error occurs

def send_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Function to send velocity commands to the drone.
    Arguments:
    vehicle: The connected dronekit vehicle instance.
    velocity_x, velocity_y, velocity_z: Velocity components in the x, y, and z directions.
    duration: Time duration for which the velocity is applied.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # Timestamp and target IDs
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Use body NED frame
        0b0000111111000111,  # Mask to enable only velocity
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # Velocity components
        0, 0, 0,  # Acceleration (not used)
        0, 0)  # Yaw (not used)
    
    for _ in range(duration * 10):  # Send the command multiple times for duration seconds
        vehicle.send_mavlink(msg)
        time.sleep(0.1)  # Sleep for 0.1 seconds

class DroneObstacleAvoidance:

    def __init__(self, connection_string):
        # Connect to the vehicle
        self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
        
        # Set obstacle avoidance distance to 3 meters (300 cm)
        self.obstacle_prevent_distance = 130  # Adjust distance threshold here

        # Altitude threshold for enabling the obstacle avoidance system
        self.target_altitude = 3.0  # Altitude in meters

        # Start the obstacle avoidance system
        self.obstacle_thread = threading.Thread(target=self.obstacle_avoidance_system)
        self.obstacle_thread.daemon = True
        self.obstacle_thread.start()

    def obstacle_avoidance_system(self):
        while True:
            try:
                # Check the current altitude
                current_altitude = self.vehicle.location.global_relative_frame.alt
                print(f"Current Altitude: {current_altitude} meters")

                # Activate obstacle avoidance only at target altitude (3 meters ± 0.2 meters tolerance)
                if abs(current_altitude - self.target_altitude) > 0.2:
                    print("Obstacle avoidance inactive - Altitude not within range.")
                    time.sleep(0.5)  # Check again after 0.5 seconds
                    continue

                # Measure distances from sensors
                front_distance = measure_distance(front_sensor)
                back_distance = measure_distance(back_sensor)
                left_distance = measure_distance(left_sensor)
                right_distance = measure_distance(right_sensor)

                print(f"Front: {front_distance} cm, Back: {back_distance} cm, Left: {left_distance} cm, Right: {right_distance} cm")

                # Ensure the drone is in GUIDED mode for obstacle avoidance
                if self.vehicle.mode != VehicleMode("GUIDED"):
                    print("Switching to GUIDED mode for obstacle avoidance")
                    self.vehicle.mode = VehicleMode("GUIDED")
                    while self.vehicle.mode != VehicleMode("GUIDED"):
                        time.sleep(0.1)

                # Obstacle detection flags
                obstacle_front = front_distance < self.obstacle_prevent_distance
                obstacle_back = back_distance < self.obstacle_prevent_distance
                obstacle_left = left_distance < self.obstacle_prevent_distance
                obstacle_right = right_distance < self.obstacle_prevent_distance

                # Decision Logic for Obstacle Avoidance
                if (obstacle_front and obstacle_back) or (obstacle_left and obstacle_right):
                    send_velocity(self.vehicle, 0, 0, 1, duration=1)
                    print("Obstacles in multiple directions - Increasing Height!")
                elif obstacle_front:
                    send_velocity(self.vehicle, -1, 0, 0, duration=1)
                    print("Obstacle detected in Front - Moving Backward!")
                elif obstacle_back:
                    send_velocity(self.vehicle, 1, 0, 0, duration=1)
                    print("Obstacle detected in Back - Moving Forward!")
                elif obstacle_left:
                    send_velocity(self.vehicle, 0, 1, 0, duration=1)
                    print("Obstacle detected on Left - Moving Right!")
                elif obstacle_right:
                    send_velocity(self.vehicle, 0, -1, 0, duration=1)
                    print("Obstacle detected on Right - Moving Left!")

                time.sleep(0.5)  # Wait half a second before the next check
            except Exception as e:
                print(f"Error in obstacle avoidance: {e}")
                break

# class DroneObstacleAvoidance:

#     def __init__(self, connection_string):
#         # Connect to the vehicle
#         self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
#         self.obstacle_prevent_distance = 130  # Default obstacle avoidance distance in cm

#         # Start the obstacle avoidance system
#         self.obstacle_thread = threading.Thread(target=self.obstacle_avoidance_system)
#         self.obstacle_thread.daemon = True
#         self.obstacle_thread.start()

#     def obstacle_avoidance_system(self):
#      while True:
#         try:
#             # Measure distances from sensors
#             front_distance = measure_distance(front_sensor)
#             back_distance = measure_distance(back_sensor)
#             left_distance = measure_distance(left_sensor)
#             right_distance = measure_distance(right_sensor)

#             print(f"Front: {front_distance} cm, Back: {back_distance} cm, Left: {left_distance} cm, Right: {right_distance} cm")

#             # Save the previous mode for potential restoration later
#             prev_mode = self.vehicle.mode

#             # Obstacle detection flags
#             obstacle_front = front_distance < self.obstacle_prevent_distance
#             obstacle_back = back_distance < self.obstacle_prevent_distance
#             obstacle_left = left_distance < self.obstacle_prevent_distance
#             obstacle_right = right_distance < self.obstacle_prevent_distance

#             # Decision Logic for Obstacle Avoidance
#             if (obstacle_front and obstacle_back) or (obstacle_left and obstacle_right):
#                 self.ensure_guided_mode()
#                 send_velocity(self.vehicle, 0, 0, 1, duration=1)
#                 print("Obstacles in multiple directions - Increasing Height!")

#             else:
#                 # Handle obstacles in individual directions
#                 if obstacle_front and not (obstacle_back or obstacle_left or obstacle_right):
#                     self.ensure_guided_mode()
#                     send_velocity(self.vehicle, -1, 0, 0, duration=1)
#                     print("Obstacle detected in Front - Moving Backward!")

#                 elif obstacle_back and not (obstacle_front or obstacle_left or obstacle_right):
#                     self.ensure_guided_mode()
#                     send_velocity(self.vehicle, 1, 0, 0, duration=1)
#                     print("Obstacle detected in Back - Moving Forward!")

#                 elif obstacle_left and not (obstacle_front or obstacle_back or obstacle_right):
#                     self.ensure_guided_mode()
#                     send_velocity(self.vehicle, 0, 1, 0, duration=1)
#                     print("Obstacle detected on Left - Moving Right!")

#                 elif obstacle_right and not (obstacle_front or obstacle_back or obstacle_left):
#                     self.ensure_guided_mode()
#                     send_velocity(self.vehicle, 0, -1, 0, duration=1)
#                     print("Obstacle detected on Right - Moving Left!")

#             # Restore previous mode if necessary
#             if self.vehicle.mode == VehicleMode("GUIDED") and prev_mode != VehicleMode("GUIDED"):
#                 self.vehicle.mode = prev_mode
#                 while self.vehicle.mode != prev_mode:
#                     time.sleep(0.1)

#             time.sleep(0.5)  # Wait half a second before the next check
#         except Exception as e:
#             print(f"Error measuring distance: {e}")
#             break

#     def ensure_guided_mode(self):
#      if self.vehicle.mode != VehicleMode("GUIDED"):
#         print("Switching to GUIDED mode")
#         self.vehicle.mode = VehicleMode("GUIDED")
#         while self.vehicle.mode != VehicleMode("GUIDED"):
#             time.sleep(0.1)  # Wait until the mode change is complete

#     def close(self):
#         self.vehicle.close()

# Main code execution
if __name__ == '__main__':
    drone_control = DroneObstacleAvoidance(args.connect)
    try:
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")
        drone_control.close()
