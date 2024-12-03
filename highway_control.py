#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive

class CombinedControlNode(Node):
    def __init__(self, world):
        super().__init__('combined_control_node')

        # Publishers and Subscribers
        self.control_publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
        self.create_subscription(Float32, '/lane_center_offset', self.control_steering, 10)

        # Publisher to send a stop flag when a stop sign is detected
        self.stop_publisher = self.create_publisher(
            Bool, # Message type
            '/carla/ego_vehicle/action_flag', # Topic name
            10) # Queue size

        # Initialize variables for speed and steering
        self.lane_offset = 0.0  # Default lane center offset
        self.stop_flag = False  # Default stop signal flag
        # Timer to publish both speed and steering together
        self.timer = self.create_timer(0.05, self.publish_control_command)

        self.ego_vehicle = None

        if world is not None:
            for actor in world.get_actors().filter('vehicle.*'):
                if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
                    self.ego_vehicle = actor
                    break  # Stop once we find the ego vehicle

            if self.ego_vehicle is None:
                print("Ego vehicle not found!")

    def control_steering(self, msg):
        """Callback for updating the lane center offset for lateral control."""
        self.lane_offset = msg.data
        self.get_logger().info(f'Received lane offset: {self.lane_offset}')

    def publish_control_command(self):
        """Publish Ackermann drive message with both speed and steering angle."""
        drive_msg = AckermannDrive()

        if self.ego_vehicle is not None:
            ego_transform = self.ego_vehicle.get_transform()
            # Access the rotation (pitch, yaw, roll)
            rotation = ego_transform.rotation
            location = ego_transform.location

        if abs(location.x - -41.2) < 4 and abs(location.y - 145.1) < 10:
            stop_msg = Bool()
            stop_msg.data = True # Set stop flag to True
            self.stop_publisher.publish(stop_msg) # Publish the stop flag
            self.get_logger().info('Hospital Reached. Asking the master to break.') # Log the detection
        
        # Adjust steering angle based on lane offset and the nature of the turn
        if abs(self.lane_offset) < 0.0002:
            res = round(rotation.yaw / 90)
            steering_angle = (rotation.yaw - res * 90.0)*0.1
            max_steering_angle = 10.0  # Smaller max steering angle for straight roads

        elif abs(self.lane_offset) < 0.04:  # Small offset, indicating a straight or nearly straight lane
            steering_angle = -self.lane_offset*0.8  # Small correction
            max_steering_angle = 0.5  # Smaller max steering angle for straight roads
        else:  # Larger offset, indicating a turn
            steering_angle = -self.lane_offset * 4  # Larger scaling for sharper turns
            max_steering_angle = 1.5  # Larger max steering angle for turns
            
        # Clamp steering angle within the max bounds
        drive_msg.steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))
        
        drive_msg.speed = 12.0  # Move forward at default speed
        
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