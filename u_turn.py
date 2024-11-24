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
        self.action_publisher = self.create_publisher(
            Bool, # Message type
            '/carla/ego_vehicle/action_flag', # Topic name
            10) # Queue size
        

    def timer_callback(self):
        current_time = time() - self.start_time
        msg = AckermannDrive()
        if current_time <= 10:
            # First XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 0 * (pi / 180) # Convert degrees to radians
            msg.speed = -30 / 3.6 # Convert kph to mps

        elif current_time <= 11:
            # Next XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 0.0 * (pi / 180)
            msg.speed = 0 / 3.6

        elif current_time <= 22.9:
            # Next XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 45.0 * (pi / 180)
            msg.speed = 30 / 3.6
 
        elif current_time <= 27.4: 
            # Next XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 0 * (pi / 180)
            msg.speed = 30 / 3.6

        elif current_time <= 31.4:
            # Next XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 33 * (pi / 180)
            msg.speed = 30 / 3.6

        else:
            # Stop after XX seconds
            msg.steering_angle = 0.0
            # msg.speed = 0.0

            action_msg = Bool()
            action_msg.data = True # Set action flag to True
            self.action_publisher.publish(action_msg) # Publish the action flag
            self.get_logger().info('U-Turn is done. Notifying the master.') # Log the detection

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    custom_control_node = CustomControlNode()
    rclpy.spin(custom_control_node)
    custom_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()