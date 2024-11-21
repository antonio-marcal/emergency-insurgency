#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from time import time
from ackermann_msgs.msg import AckermannDrive
from math import pi
from std_msgs.msg import Bool

class CustomControlNode(Node):
    def __init__(self):
        super().__init__('custom_control_node')
        self.publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time()

        # Publisher to send a stop flag when u-turn is over
        self.stop_publisher = self.create_publisher(
            Bool, # Message type
            '/carla/ego_vehicle/action_flag', # Topic name
            10) # Queue size
        

    def timer_callback(self):
        current_time = time() - self.start_time
        msg = AckermannDrive()
        if current_time <= 2:
            # First 10 seconds: 30 kph (approx 8.33 m/s), 5 degrees steering angle
            msg.steering_angle = 0 * (pi / 180) # Convert degrees to radians
            msg.speed = -30 / 3.6 # Convert kph to mps
        elif current_time <= 10:
            # Next 10 seconds: 30 kph (approx 8.33 m/s), 0 degrees steering angle
            msg.steering_angle = 25.0 * (pi / 180)
            msg.speed = 30 / 3.6
        elif current_time <= 17:
            # Next 10 seconds: 30 kph (approx 8.33 m/s), 0 degrees steering angle
            msg.steering_angle = 0 * (pi / 180)
            msg.speed = 30 / 3.6
        elif current_time <= 25:
            # Next 10 seconds: 30 kph (approx 8.33 m/s), 0 degrees steering angle
            msg.steering_angle = 25 * (pi / 180)
            msg.speed = 30 / 3.6
        else:
            # Stop after 20 seconds
            msg.steering_angle = 0.0  # Convert degrees to radians
            msg.speed = 0.0 # Convert kph to mps

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    custom_control_node = CustomControlNode()
    rclpy.spin(custom_control_node)
    custom_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()