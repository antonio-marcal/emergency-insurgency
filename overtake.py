#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist


class CombinedControlNode(Node):
    def __init__(self, world):
            super().__init__('combined_control_node')
            # Publishers and Subscribers
            self.control_publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
            self.create_subscription(Twist, '/carla/ego_vehicle/cmd_vel', self.update_velocity, 10)

            # Publisher to send a stop flag when u-turn is over
            self.action_publisher = self.create_publisher(
                Bool, # Message type
                '/carla/ego_vehicle/action_flag', # Topic name
                10) # Queue size

            # Timer to publish both speed and steering together
            self.timer = self.create_timer(0.1, self.publish_control_command)

            self.twist = Twist()
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

            self.ego_vehicle = None

            for actor in world.get_actors().filter('vehicle.*'):
                if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
                    self.ego_vehicle = actor
                    break  # Stop once we find the ego vehicle

            if self.ego_vehicle is None:
                print("Ego vehicle not found!")

        #    self.timer2 = self.create_timer(3, self.refresh_overtake)

    def update_velocity(self, msg):
        """Callback for updating the twist info from semantic camera."""
        self.twist = msg

    def publish_control_command(self):
            """Publish Ackermann drive message with both speed and steering angle."""
            drive_msg = AckermannDrive()

            drive_msg.speed = self.twist.linear.x
            drive_msg.steering_angle = self.twist.angular.z
            # Publish the combined control message
            self.control_publisher.publish(drive_msg)

            self.check_stop()

    def check_stop(self):
        # Get the Transform (location and rotation)
        ego_transform = self.ego_vehicle.get_transform()

        # Access the location (x, y, z)
        location = ego_transform.location
        if location.x > 92:
            action_msg = Bool()
            action_msg.data = True # Set action flag to True
            self.action_publisher.publish(action_msg) # Publish the action flag
            self.get_logger().info('Overtake is done. Notifying the master.') # Log the detection
             


def main(args=None):
   rclpy.init(args=args)
   node = CombinedControlNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

