#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import numpy as np
import os
import cv2

class SemanticCameraNode(Node):
  def __init__(self, world):
      super().__init__('semantic_camera_node')
      self.subscription = self.create_subscription(
          Image,
          '/carla/ego_vehicle/semantic_segmentation_front/image',
          self.listener_callback,
          10
      )
      self.bridge = CvBridge()
      self.lane_offset_publisher = self.create_publisher(Float32, '/lane_center_offset',10)
        
      self.image_counter = 1
      # Update the path to a valid location (e.g., inside the current user's directory)
      self.image_path = '/home/cae-user/Documents/highway_camera_output/'  # Change this path if needed
      os.makedirs(self.image_path, exist_ok=True)

      self.ego_vehicle = None

      if world is not None:
        for actor in world.get_actors().filter('vehicle.*'):
            if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
                self.ego_vehicle = actor
                break  # Stop once we find the ego vehicle

        if self.ego_vehicle is None:
            print("Ego vehicle not found!")
      
  def listener_callback(self, msg):
      cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
      filename = os.path.join(self.image_path, f'image_{self.image_counter}.png')
      cv2.imwrite(filename, cv_image)
      self.image_counter += 1
      cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
      # Extract Blue and Red channels (Green is not needed)
      blue_channel = cv_image[:, :, 0]
      red_channel = cv_image[:, :, 2]
      # Create masks for lane lines and vehicles based on specific color codes
      lane_line_mask = red_channel == 157 # Lane lines are colored (BGR): (50, 234, 157)
      # Compute the lane center offset
      _, lane_center_offset = self.compute_centerline(lane_line_mask)
      # Publish the lane center offset
      lane_offset_msg = Float32()
      lane_offset_msg.data = lane_center_offset
      self.lane_offset_publisher.publish(lane_offset_msg)

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

def main(args=None):
  rclpy.init(args=args)
  node = SemanticCameraNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()