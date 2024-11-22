#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist


class CombinedControlNode(Node):
    def __init__(self):
            super().__init__('combined_control_node')
            # Publishers and Subscribers
            self.control_publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
        #    self.create_subscription(Float32, '/lane_center_offset', self.control_steering, 10)
        #    self.create_subscription(Bool, '/vehicle_in_front', self.update_vehicle_in_front_status, 10)
            self.create_subscription(Twist, '/carla/ego_vehicle/cmd_vel', self.update_velocity, 10)

        
            # self.was_vehicle_in_front = False

            # Initialize variables for speed and steering
            # self.lane_offset = 0.0  # Default lane center offset
            # self.vehicle_in_front = False  # Indicates if a vehicle is detected in front
            # self.returning_to_position = False  # Tracks if the vehicle is returning to its original position

            # Timer to publish both speed and steering together
            self.timer = self.create_timer(0.1, self.publish_control_command)

            self.twist = Twist()
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        #    self.timer2 = self.create_timer(3, self.refresh_overtake)

    def update_velocity(self, msg):
        """Callback for updating the twist info from semantic camera."""
        self.twist = msg

       

#    def refresh_overtake(self):
#        self.was_vehicle_in_front = False

#    def control_steering(self, msg):
#        """Callback for updating the lane center offset for lateral control."""
#        self.lane_offset = msg.data
#        self.get_logger().info(f'Received lane offset: {self.lane_offset}')

#    def update_vehicle_in_front_status(self, msg):
#        """Callback for updating the status of vehicle detection."""
#        self.was_vehicle_in_front = self.vehicle_in_front or self.was_vehicle_in_front
#        self.vehicle_in_front = msg.data

#        if not self.vehicle_in_front and self.was_vehicle_in_front:
#            # If the vehicle was detected but is no longer in view, start returning to position
#            self.returning_to_position = True
#        elif self.vehicle_in_front:
#            # If a vehicle is detected, stop the return-to-position process
#            self.returning_to_position = False

#        self.get_logger().info(f'Vehicle in front: {self.vehicle_in_front}')

    def publish_control_command(self):
            """Publish Ackermann drive message with both speed and steering angle."""
            drive_msg = AckermannDrive()

        #    if self.vehicle_in_front:
        #        # Vehicle in front: turn left
        #        self.get_logger().info("Vehicle in front detected! Turning left.")
        #        drive_msg.speed = 7.0  # Reduce speed for safety
        #        drive_msg.steering_angle = 2.0  # Turn left
        #    elif self.returning_to_position:
        #        # Returning to original position
        #        self.get_logger().info("Vehicle clear. Turning right to original position.")
        #        drive_msg.speed = 5.0
        #        drive_msg.steering_angle = -1.0  # Turn right to return to original position

        #        # Stop returning once the angle is reset
        #        self.returning_to_position = False
        #    else:
        #         # Clear road: Maintain straight driving
        #         # self.get_logger().info("Clear road ahead. Driving straight.")
        #         drive_msg.speed = 7.0  # Default cruising speed
        #         #    drive_msg.steering_angle = 0.0  # Straight driving

        #    # Log the control command for debugging
        #    self.get_logger().info(
        #        f'Publishing control command: speed={drive_msg.speed}, steering_angle={drive_msg.steering_angle}'
        #    )


            drive_msg.speed = self.twist.linear.x
            drive_msg.steering_angle = self.twist.angular.z
            # Publish the combined control message
            self.control_publisher.publish(drive_msg)


def main(args=None):
   rclpy.init(args=args)
   node = CombinedControlNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

