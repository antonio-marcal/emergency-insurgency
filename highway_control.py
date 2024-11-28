#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive

class CombinedControlNode(Node):
    def __init__(self):
        super().__init__('combined_control_node')

        # Publishers and Subscribers
        self.control_publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
        self.create_subscription(Float32, '/lane_center_offset', self.control_steering, 10)

        # Initialize variables for speed and steering
        self.lane_offset = 0.0  # Default lane center offset
        self.stop_flag = False  # Default stop signal flag
        # Timer to publish both speed and steering together
        self.timer = self.create_timer(0.05, self.publish_control_command)

    def control_steering(self, msg):
        """Callback for updating the lane center offset for lateral control."""
        self.lane_offset = msg.data
        self.get_logger().info(f'Received lane offset: {self.lane_offset}')

    def publish_control_command(self):
        """Publish Ackermann drive message with both speed and steering angle."""
        drive_msg = AckermannDrive()
        
        # Adjust steering angle based on lane offset and the nature of the turn
        if abs(self.lane_offset) < 0.05:  # Small offset, indicating a straight or nearly straight lane
            steering_angle = -self.lane_offset * 0.8  # Small correction
            max_steering_angle = 0.5  # Smaller max steering angle for straight roads
        elif abs(self.lane_offset)<0.1:
            steering_angle = -self.lane_offset * 2  # Larger scaling for sharper turns
            max_steering_angle = 1.3  # Larger max steering angle for turns
        else:  # Larger offset, indicating a turn
            steering_angle = -self.lane_offset * 5  # Larger scaling for sharper turns
            max_steering_angle = 1.3  # Larger max steering angle for turns
            
        # Clamp steering angle within the max bounds
        drive_msg.steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))
        
        drive_msg.speed = 6.0  # Move forward at default speed
        
        # Log the control command for debugging
        #self.get_logger().info(f'Publishing control command: speed={drive_msg.speed}, steering_angle={drive_msg.steering_angle}')
        
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