#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool
from transforms3d.euler import quat2euler
import math
import pandas as pd
import numpy as np
import csv
import time




class PurePursuitController(Node):
    def __init__(self, filename, angle, dist, speed):
        super().__init__('pure_pursuit_controller')

        self.max_steering_angle = np.radians(angle) # Maximum steering angle in radians
        self.wheelbase = 4.5 # Wheelbase of the Ford Ambulance in meters
        self.lookahead_distance = dist # Increased look-ahead distance in meters
        self.target_speed = speed # Increased speed of the vehicle in meters per second

        # Load waypoints from the specified CSV file
        waypoints = pd.read_csv(filename)
        self.waypoint_x = waypoints['x'].to_numpy()
        self.waypoint_y = waypoints['y'].to_numpy()

        # Initialize vehicle state
        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_yaw = None
        self.rows = []

        # Subscribers for vehicle odometry and speed
        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry',
        self.odometry_callback, 10)
        self.create_subscription(Float32, '/carla/ego_vehicle/speedometer',
        self.speed_callback, 10)

        # Publisher for vehicle control commands
        self.control_publisher = self.create_publisher(AckermannDrive,
        '/carla/ego_vehicle/ackermann_cmd', 10)

        # Timer to run the control loop at 100 Hz
        self.control_timer = self.create_timer(0.01, self.run_control_loop)

        # Publisher to send a flag when waypoints are done
        self.stop_publisher = self.create_publisher(
            Bool, # Message type
            '/carla/ego_vehicle/waypoints_done', # Topic name
            10) # Queue size

    def odometry_callback(self, msg):
        # Update vehicle position and orientation from odometry data
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        _, _, self.vehicle_yaw = quat2euler([msg.pose.pose.orientation.w,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z])
    def speed_callback(self, msg):
        self.vehicle_speed = msg.data
    
    def run_control_loop(self):
        # Ensure we have received initial odometry data
        if self.vehicle_x is None or self.vehicle_y is None:
            return
        
        # Calculate the steering angle and publish the control command
        steering_angle, waypoint_index = self.calculate_steering_angle(self.lookahead_distance)
        self.publish_control_command(steering_angle)

        if  (self.waypoint_x.size - waypoint_index) <= .11 * self.waypoint_x.size:
            stop_msg = Bool()
            stop_msg.data = True # Set stop flag to True
            self.get_logger().info('Waypoints are over!!! Notifying the master.') # Log the detection
            self.stop_publisher.publish(stop_msg) # Publish the stop flag  

        # Record the current state for logging purposes
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        self.rows.append([current_time, self.vehicle_x, self.vehicle_y, self.vehicle_speed])

    def calculate_steering_angle(self, lookahead_distance):
        # Find the nearest waypoint to the current vehicle position
        nearest_waypoint_index = self.find_nearest_waypoint(self.vehicle_x, self.vehicle_y)

        target_x = self.waypoint_x[nearest_waypoint_index]
        target_y = self.waypoint_y[nearest_waypoint_index]
        
        # Calculate the heading error and steering angle using the pure pursuit formula
        heading_error = math.atan2(target_y - self.vehicle_y, target_x - self.vehicle_x) -self.vehicle_yaw
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(heading_error) /
        lookahead_distance, 1.0)
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        return steering_angle, nearest_waypoint_index
    
    def find_nearest_waypoint(self, x, y):
        # Calculate the Euclidean distances to all waypoints and return the index of the nearest one
        distances = np.hypot(self.waypoint_x - x, self.waypoint_y - y)
        return np.argmin(distances)
    
    def publish_control_command(self, steering_angle):
        # Create and publish the control command
        control_command = AckermannDrive()
        control_command.steering_angle = steering_angle
        control_command.speed = self.target_speed
        self.control_publisher.publish(control_command)

    def __del__(self):
        # Write the logged vehicle states to a CSV file
        with open("output.csv", 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['time', 'x', 'y', 'speed'])
            csvwriter.writerows(self.rows)

def main():
    rclpy.init()
    filename = 'waypoints_phase1_expected.csv' # Specify the CSV file with waypoints
    pure_pursuit_controller = PurePursuitController(filename)
    
    try:
        rclpy.spin(pure_pursuit_controller)
    except KeyboardInterrupt:
        pass
    pure_pursuit_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    