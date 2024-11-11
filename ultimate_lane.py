#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import numpy as np

class SemanticCameraNode(Node):
    def __init__(self):
        super().__init__('semantic_camera_node')
        # Subscribe to the semantic segmentation image topic
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.listener_callback,
            10
        )
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        # Publishers for lane center offset and stop signal
        self.lane_offset_publisher = self.create_publisher(Float32, '/lane_center_offset', 10)
        self.stop_signal_publisher = self.create_publisher(Bool, '/stop_signal', 10)

    def listener_callback(self, msg):
        """
        Callback function that processes incoming images, computes the lane center offset,
        detects vehicles in front, and publishes the results.
        """
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Extract Blue and Red channels (Green is not needed)
        blue_channel = cv_image[:, :, 0]
        red_channel = cv_image[:, :, 2]
        # Create masks for lane lines and vehicles based on specific color codes
        lane_line_mask = red_channel == 157 # Lane lines are colored (BGR): (50, 234, 157)
        vehicle_mask = blue_channel == 142 # Vehicles are colored (BGR): (142, 0, 0)
        # Compute the lane center offset
        _, lane_center_offset = self.compute_centerline(lane_line_mask)
        # Publish the lane center offset
        lane_offset_msg = Float32()
        lane_offset_msg.data = lane_center_offset
        self.lane_offset_publisher.publish(lane_offset_msg)
        # Detect if a vehicle is in front and publish the stop signal
        stop_signal = self.detect_vehicle_in_front(vehicle_mask)
        stop_signal_msg = Bool()
        stop_signal_msg.data = bool(stop_signal)
        self.stop_signal_publisher.publish(stop_signal_msg)

    def compute_centerline(self, lane_line_mask):
        """
        Compute the centerline of the lane and calculate the offset from the image center.
        Args:
        lane_line_mask (numpy.ndarray): Binary mask where lane lines are True.
        Returns:
        center_points (list): List of (x, y) tuples representing the lane centerline.
        lane_center_offset (float): Normalized offset from the image center (-1 to 1).
        """
        height, width = lane_line_mask.shape
        center_points = []
        image_center_x = width // 2
        # Iterate over each row to find the lane center
        for y in range(height):
            x_indices = np.where(lane_line_mask[y, :])[0]
            if len(x_indices) >= 2:
                # Separate indices into left and right of image center
                x_left = x_indices[x_indices < image_center_x]
                x_right = x_indices[x_indices > image_center_x]
                if len(x_left) > 0 and len(x_right) > 0:
                    x_leftmost = x_left[-1] # Rightmost point on the left lane line
                    x_rightmost = x_right[0] # Leftmost point on the right lane line
                    x_center = int((x_leftmost + x_rightmost) / 2)
                else:
                    # If only one side is detected, assume center is at image center
                    x_center = image_center_x
                center_points.append((x_center, y))
            else:
                # If no lane lines are detected, use image center
                x_center = image_center_x
                center_points.append((x_center, y))
            
        # Calculate the average lane center offset from the bottom half of the image
        if center_points:
            bottom_half_points = [pt for pt in center_points if pt[1] > height * 0.5]
            if bottom_half_points:
                avg_x_center = np.mean([pt[0] for pt in bottom_half_points])
                # Normalize offset to range [-1, 1]
                lane_center_offset = (avg_x_center - image_center_x) / (width / 2)
            else:
                lane_center_offset = 0.0
        else:
            lane_center_offset = 0.0
        return center_points, lane_center_offset
    
    def detect_vehicle_in_front(self, vehicle_mask):
        """
        Detect if there is a vehicle directly in front within a specified region of
        interest.
        Args:
        vehicle_mask (numpy.ndarray): Binary mask where vehicles are True.
        Returns:
        bool: True if a vehicle is detected in front, False otherwise.
        """
        height, width = vehicle_mask.shape
        
        # Define ROI: central 10% width, bottom 30% height
        roi_top = int(height * 0.7)
        roi_bottom = height
        roi_left = int(width * 0.48)
        roi_right = int(width * 0.52)
        
        vehicle_roi = vehicle_mask[roi_top:roi_bottom, roi_left:roi_right]
        
        # Return True if any vehicles are detected in the ROI
        return np.any(vehicle_roi)

def main(args=None):
    rclpy.init(args=args)
    # Create and run the semantic camera node
    node = SemanticCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()