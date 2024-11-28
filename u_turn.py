#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from time import time
from ackermann_msgs.msg import AckermannDrive
from math import pi
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import carla

class CustomControlNode(Node):
    def __init__(self, world):
        super().__init__('custom_control_node')
        self.publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
        self.create_subscription(Twist, '/carla/ego_vehicle/cmd_vel', self.update_velocity, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time()

        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        self.turn_start = -1
        self.u_turned = False


        # Publisher to send a stop flag when u-turn is over
        self.action_publisher = self.create_publisher(
            Bool, # Message type
            '/carla/ego_vehicle/action_flag', # Topic name
            10) # Queue size
        

        self.ego_vehicle = None

        for actor in world.get_actors().filter('vehicle.*'):
            if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
                self.ego_vehicle = actor
                break  # Stop once we find the ego vehicle

        if self.ego_vehicle is None:
            print("Ego vehicle not found!")
        

    def timer_callback(self):
        current_time = time() - self.start_time
        msg = AckermannDrive()
        ego_transform = self.ego_vehicle.get_transform()

        # Access the location (x, y, z)
        location = ego_transform.location
        # Access the rotation (pitch, yaw, roll)
        rotation = ego_transform.rotation

        if current_time <= 10:
            # First XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 0 * (pi / 180) # Convert degrees to radians
            msg.speed = -30 / 3.6 # Convert kph to mps

        elif current_time <= 11:
            # Next XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 0.0 * (pi / 180)
            msg.speed = 0 / 3.6
            

        elif (98 < rotation.yaw <180 or -180 <rotation.yaw <-87)  and not self.u_turned:
            # Next XX seconds: XX kph XX degrees steering angle
            msg.steering_angle = 45.0 * (pi / 180)
            msg.speed = 30 / 3.6
            

        elif location.y < 62:
            self.u_turned = True
            # Next XX seconds: XX kph XX degrees steering angle
            msg.speed = self.twist.linear.x
            msg.steering_angle = self.twist.angular.z
            # msg.steering_angle = 0.0 * (pi / 180)
            # msg.speed = 30 / 3.6
            
        else:
            print("Start turning")
            msg.steering_angle = 33 * (pi / 180)
            msg.speed = 30 / 3.6

            if self.turn_start < 0:
                self.turn_start = time()

            if time() - self.turn_start > 4:

                # Stop after XX seconds
                msg.steering_angle = 0.0
                # msg.speed = 0.0

                action_msg = Bool()
                action_msg.data = True # Set action flag to True
                self.action_publisher.publish(action_msg) # Publish the action flag
                self.get_logger().info('U-Turn is done. Notifying the master.') # Log the detection


        # elif current_time <= 31.4:
        #     # Next XX seconds: XX kph XX degrees steering angle
        #     msg.steering_angle = 33 * (pi / 180)
        #     msg.speed = 30 / 3.6
                
        self.publisher.publish(msg)

    def update_velocity(self, msg):
        """Callback for updating the twist info from semantic camera."""
        self.twist = msg

def main(args=None):
    rclpy.init(args=args)
    custom_control_node = CustomControlNode()
    rclpy.spin(custom_control_node)
    custom_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()