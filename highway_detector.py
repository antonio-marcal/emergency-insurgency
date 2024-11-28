import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class HighwayDetectionNode(Node):
    def __init__(self, world):
        super().__init__('highway_detector') # Initialize the node with a name
       
        # Publisher to send a stop flag when a stop sign is detected
        self.stop_publisher = self.create_publisher(
            Bool, # Message type
            '/carla/ego_vehicle/action_flag', # Topic name
            10) # Queue size
        
        self.detection_timer = self.create_timer(0.04, self.detect_highway)

        self.ego_vehicle = None

        for actor in world.get_actors().filter('vehicle.*'):
            if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
                self.ego_vehicle = actor
                break  # Stop once we find the ego vehicle

        if self.ego_vehicle is None:
            print("Ego vehicle not found!")

    def detect_highway(self):

        ego_transform = self.ego_vehicle.get_transform()

        # Access the location (x, y, z)
        location = ego_transform.location
        # Access the rotation (pitch, yaw, roll)
        rotation = ego_transform.rotation

        if location.y < 0 and 84 < abs(rotation.yaw) < 91:
            stop_msg = Bool()
            stop_msg.data = True # Set stop flag to True
            self.get_logger().info('Vehicle is centered in the higway. Notifying the master.') # Log the detection
            self.stop_publisher.publish(stop_msg) # Publish the stop flag
    


def main(args=None):
    rclpy.init(args=args) # Initialize the ROS2 Python library
    node = HighwayDetectionNode() # Create an instance of the YoloDetectionNode
    rclpy.spin(node) # Keep the node running
    node.destroy_node() # Destroy the node when shutting down
    rclpy.shutdown() # Shutdown the ROS2 Python library

if __name__ == '__main__':
    main() # Execute the main function when the script is run