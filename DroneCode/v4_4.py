import argparse
import time
import threading
from dronekit import connect, VehicleMode
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
        return round(sensor.distance * 100, 2)  # Convert to centimeters
    except Exception as e:
        print(f"Error reading sensor: {e}")
        return None


class DroneObstacleAvoidance:

    def __init__(self, connection_string):
        # Connect to the vehicle
        self.vehicle = connect(connection_string, baud=921600, wait_ready=True)
        self.obstacle_prevent_enabled = False
        self.obstacle_prevent_distance = 200  # Default obstacle avoidance distance in cm
        # self.latest_obstacle_status = "All Clear"  # Store the latest status

        # Start the obstacle avoidance system
        self.obstacle_thread = threading.Thread(target=self.obstacle_avoidance_system)
        self.obstacle_thread.daemon = True
        self.obstacle_thread.start()

    def toggle_avoidance_mode(self):
        self.obstacle_prevent_enabled = not self.obstacle_prevent_enabled
        print(f"Obstacle avoidance {'enabled' if self.obstacle_prevent_enabled else 'disabled'}.")

    def obstacle_avoidance_system(self):
     while True:
        try:
            if self.obstacle_prevent_enabled:
                # Get safe distance from the slider (or use a fixed value here for simplicity)
                self.obstacle_prevent_distance = 200  # Example: 200 cm

                # Measure distances from sensors
                front_distance = measure_distance(front_sensor)
                back_distance = measure_distance(back_sensor)
                left_distance = measure_distance(left_sensor)
                right_distance = measure_distance(right_sensor)

                # Print the sensor distance data
                # print(f"Front Distance: {front_distance} cm")
                # print(f"Back Distance: {back_distance} cm")
                # print(f"Left Distance: {left_distance} cm")
                # print(f"Right Distance: {right_distance} cm")

                # Get the drone's current velocity
                current_velocity = self.vehicle.velocity  # (vx, vy, vz) in m/s
                speed_magnitude = (current_velocity[0] ** 2 + current_velocity[1] ** 2 + current_velocity[2] ** 2) ** 0.5

                # Obstacle detection flags
                obstacle_front = front_distance < self.obstacle_prevent_distance
                obstacle_back = back_distance < self.obstacle_prevent_distance
                obstacle_left = left_distance < self.obstacle_prevent_distance
                obstacle_right = right_distance < self.obstacle_prevent_distance

                # Decision Logic for Obstacle Avoidance
                # obstacle_status = "All Clear"

                # Increase height if obstacles are detected in multiple directions
                if (obstacle_front and obstacle_back) or (obstacle_left and obstacle_right) or \
                        (obstacle_front and obstacle_back and obstacle_left) or \
                        (obstacle_front and obstacle_back and obstacle_right) or \
                        (obstacle_left and obstacle_right and obstacle_back) or \
                        (obstacle_left and obstacle_right and obstacle_front) or \
                        (obstacle_front and obstacle_back and obstacle_left and obstacle_right):
                    self.vehicle.simple_goto(self.vehicle.location.global_frame.lat,
                                             self.vehicle.location.global_frame.lon,
                                             self.vehicle.location.global_frame.alt + 1)
                    print("Obstacles in multiple directions - Increasing Height!")
                else:
                    # Move backward or sideways depending on obstacle position
                    if obstacle_front and not (obstacle_back or obstacle_left or obstacle_right):
                        if speed_magnitude > 2:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat,
                                                     self.vehicle.location.global_frame.lon - 1.5,
                                                     self.vehicle.location.global_frame.alt)
                        else:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat,
                                                     self.vehicle.location.global_frame.lon - 1,
                                                     self.vehicle.location.global_frame.alt)
                        print("Obstacle detected in Front - Moving Backward!")

                    elif obstacle_back and not (obstacle_front or obstacle_left or obstacle_right):
                        if speed_magnitude > 2:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat,
                                                     self.vehicle.location.global_frame.lon + 1.5,
                                                     self.vehicle.location.global_frame.alt)
                        else:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat,
                                                     self.vehicle.location.global_frame.lon + 1,
                                                     self.vehicle.location.global_frame.alt)
                        print("Obstacle detected in Back - Moving Forward!") 

                    elif obstacle_left and not (obstacle_front or obstacle_back or obstacle_right):
                        if speed_magnitude > 2:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat + 1.5,
                                                     self.vehicle.location.global_frame.lon,
                                                     self.vehicle.location.global_frame.alt)
                        else:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat + 1,
                                                     self.vehicle.location.global_frame.lon,
                                                     self.vehicle.location.global_frame.alt)
                        print("Obstacle detected on Left - Moving Right!") 
 
                    elif obstacle_right and not (obstacle_front or obstacle_back or obstacle_left):
                        if speed_magnitude > 2:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat - 1.5,
                                                     self.vehicle.location.global_frame.lon,
                                                     self.vehicle.location.global_frame.alt)
                        else:
                            self.vehicle.simple_goto(self.vehicle.location.global_frame.lat - 1,
                                                     self.vehicle.location.global_frame.lon,
                                                     self.vehicle.location.global_frame.alt)
                        print("Obstacle detected on Right - Moving Left!")

                # If velocity is too low, stop and maintain safe distance
                # if speed_magnitude < 2:
                #  self.vehicle.velocity = (0, 0, 0)
                #  obstacle_status += " | Low speed - Maintaining Safe Distance"

                # Update the latest obstacle status
                # self.latest_obstacle_status = obstacle_status
                # print(f"Obstacle Status: {self.latest_obstacle_status}")

            time.sleep(0.5)  # Wait half a second before the next check
        except Exception as e:
            print(f"Error measuring distance: {e}")
            break

    def close(self):
        self.vehicle.close()


# Main code execution
if __name__ == '__main__':
    drone_control = DroneObstacleAvoidance(args.connect)
    try:
        while True:
            # Example of enabling and disabling obstacle avoidance mode
            user_input = input("Enter 'e' to activate obstacle avoidance, 'd' to deactivate: ").strip().lower()
            if user_input == 'enable':
                drone_control.toggle_avoidance_mode()
            elif user_input == 'disable':
                drone_control.toggle_avoidance_mode()
            else:
                print("Invalid input. Please type 'enable' or 'disable'.")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")
        drone_control.close()
