#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from transforms3d.euler import quat2euler
import math
import pandas as pd
import numpy as np
import csv
import time

MAX_STEERING_ANGLE = np.radians(40) # Maximum steering angle in radians
WHEELBASE = 2.6 # Wheelbase of the Harley-Davidson Low Rider motorcycle in meters
LOOKAHEAD_DISTANCE = 15.0 # Increased look-ahead distance in meters
VEHICLE_SPEED = 6.25 # Increased speed of the vehicle in meters per second


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Initialize vehicle state
        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_yaw = None
        self.vehicle_speed = None
        self.rows = []

        # Subscribers for vehicle odometry and speed
        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry',
        self.odometry_callback, 10)
        self.create_subscription(Float32, '/carla/ego_vehicle/speedometer',
        self.speed_callback, 10)

        

        # Timer to run the control loop at 20 Hz
        self.control_timer = self.create_timer(0.01, self.run_control_loop)

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

        # Record the current state for logging purposes
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        self.rows.append([current_time, self.vehicle_x, self.vehicle_y, self.vehicle_speed])
    
    def __del__(self):
        # Write the logged vehicle states to a CSV file
        with open("trajectory_overtake_3.3.csv", 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['time', 'x', 'y', 'speed'])
            csvwriter.writerows(self.rows)

def main():
    rclpy.init()
    pure_pursuit_controller = PurePursuitController()
    
    try:
        rclpy.spin(pure_pursuit_controller)
    except KeyboardInterrupt:
        pass
    pure_pursuit_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    