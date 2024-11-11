#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive

class CombinedControlNode(Node):
    def __init__(self):
        super().__init__('combined_control_node')
        # Publishers and Subscribers
        self.control_publisher = self.create_publisher(AckermannDrive,
        '/carla/ego_vehicle/ackermann_cmd', 10)
        self.create_subscription(Float32, '/lane_center_offset', self.control_steering, 10)
        self.create_subscription(Bool, '/stop_signal', self.control_speed, 10)
        
        # Initialize variables for speed and steering
        self.lane_offset = 0.0 # Default lane center offset
        self.stop_flag = False # Default stop signal flag
        
        # Timer to publish both speed and steering together
        self.timer = self.create_timer(0.1, self.publish_control_command)

    def control_steering(self, msg):
        """Callback for updating the lane center offset for lateral control."""
        self.lane_offset = msg.data
        self.get_logger().info(f'Received lane offset: {self.lane_offset}')
    
    def control_speed(self, msg):
        """Callback for updating the stop signal for longitudinal control."""
        self.stop_flag = msg.data
        self.get_logger().info(f'Received stop signal: {self.stop_flag}')

    def publish_control_command(self):
        """Publish Ackermann drive message with both speed and steering angle."""
        drive_msg = AckermannDrive()
        
        # Handle steering based on lane offset
        steering_angle = -self.lane_offset * 0.5 # Adjust scaling factor if needed
        drive_msg.steering_angle = steering_angle
        
        # Handle speed based on stop signal
        if self.stop_flag:
            drive_msg.speed = 0.0 # Stop if stop signal is active
        else:
            drive_msg.speed = 10.0 # Move forward at default speed
        
        # Log the control command for debugging
        self.get_logger().info(f'Publishing control command: speed={drive_msg.speed}, steering_angle={drive_msg.steering_angle}')
        
        # Publish the combined control message
        self.control_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CombinedControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()