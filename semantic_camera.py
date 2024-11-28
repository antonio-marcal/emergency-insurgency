#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
import time


class SemanticCameraNode(Node):
    def __init__(self, dumb_mode = False, world = None):
        super().__init__('semantic_camera_node')

        # Subscriptions and publishers
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.lane_offset_publisher = self.create_publisher(Float32, '/lane_center_offset', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/carla/ego_vehicle/cmd_vel', 10)
        self.vehicle_in_front_publisher = self.create_publisher(Bool, '/vehicle_in_front', 10)

        self.image_counter = 1
        self.image_path = '/home/cae-user/Documents/semantic_camera_output/'  # Change this path if needed
        os.makedirs(self.image_path, exist_ok=True)

        # State variables
        self.vehicle_in_front = False
        self.stage = -1
        self.timeout = 0

        self.dumb_mode = dumb_mode

        self.ego_vehicle = None

        if world is not None:
            for actor in world.get_actors().filter('vehicle.*'):
                if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
                    self.ego_vehicle = actor
                    break  # Stop once we find the ego vehicle

            if self.ego_vehicle is None:
                print("Ego vehicle not found!")

    def listener_callback(self, msg):
        # Convert image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        filename = os.path.join(self.image_path, f'image_{self.image_counter}.png')
        cv2.imwrite(filename, cv_image)
        self.image_counter += 1
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Extract Blue and Red channels
        blue_channel = cv_image[:, :, 0]
        red_channel = cv_image[:, :, 2]

        # Masks for lane lines and vehicles
        lane_line_mask = red_channel == 157  # Lane lines color (BGR): (50, 234, 157)
        vehicle_mask = blue_channel == 142  # Vehicles color (BGR): (142, 0, 0)

        # Compute lane center offset
        _, lane_center_offset = self.compute_centerline(lane_line_mask)
        lane_offset_msg = Float32()
        lane_offset_msg.data = lane_center_offset
        self.lane_offset_publisher.publish(lane_offset_msg)

        # Detect if a vehicle is in front and publish the detection status
        vehicle_in_front = self.detect_vehicle_in_front(vehicle_mask)
        vehicle_in_front_msg = Bool()
        vehicle_in_front_msg.data = bool(vehicle_in_front)
        self.vehicle_in_front_publisher.publish(vehicle_in_front_msg)

        if self.ego_vehicle is not None:
            ego_transform = self.ego_vehicle.get_transform()
            # Access the rotation (pitch, yaw, roll)
            rotation = ego_transform.rotation


        # State machine for lane change and return
        if vehicle_in_front and not self.stage >= 0 and not self.dumb_mode:
            self.vehicle_in_front = True
            self.stage = 0
            self.get_logger().info("Vehicle detected! Turning left.")
            self.perform_lane_change(left_turn=True)
            self.timeout = time.time()

        elif self.stage == 0:
            if rotation.yaw<=-20:
                # Transition from obstacle detected to left lane state
                self.vehicle_in_front = False
                self.stage = 1

                self.get_logger().info("Straightning.")
                self.straigthen_lane(4.0,-1.0)
                self.timeout = time.time()
            
        elif self.stage == 1:
            if rotation.yaw>=-16.4:
                self.get_logger().info("Vehicle cleared. Maitaining Left lane.")
                self.stage = 2
                self.timeout = time.time()
                

        elif self.stage == 2:
            
            if time.time() - self.timeout > 11:
                self.stage = 3
                # Continue returning to the lane center
                self.get_logger().info("Returning to right lane.")
                self.perform_lane_change(left_turn=False)
                self.timeout = time.time()
            else:
                self.maintain_lane(lane_center_offset, 9.0)

        elif self.stage == 3:
            if rotation.yaw>=20:
                self.stage = 4
                self.timeout = time.time()
                self.get_logger().info("Vehicle cleared. Maitaining Right lane.")
                self.straigthen_lane(4.0,1.0)

        elif self.stage == 4:
            if rotation.yaw<=16.4:
                self.stage = -1

        else:
            # Maintain the lane
            # self.get_logger().info("Clear road ahead. Maintaining lane.")
            self.maintain_lane(lane_center_offset, 10.0)

    def compute_centerline(self, lane_line_mask):
        """
        Compute lane centerline and offset from image center.
        """
        height, width = lane_line_mask.shape
        center_points = []
        image_center_x = width // 2

        for y in range(height):
            x_indices = np.where(lane_line_mask[y, :])[0]
            if len(x_indices) >= 2:
                x_left = x_indices[x_indices < image_center_x]
                x_right = x_indices[x_indices > image_center_x]
                if len(x_left) > 0 and len(x_right) > 0:
                    x_center = int((x_left[-1] + x_right[0]) / 2)
                else:
                    x_center = image_center_x
                center_points.append((x_center, y))
            else:
                center_points.append((image_center_x, y))

        if center_points:
            bottom_half_points = [pt for pt in center_points if pt[1] > height * 0.5]
            avg_x_center = np.mean([pt[0] for pt in bottom_half_points]) if bottom_half_points else image_center_x
            lane_center_offset = (avg_x_center - image_center_x) / (width / 2)
        else:
            lane_center_offset = 0.0

        return center_points, lane_center_offset

    def detect_vehicle_in_front(self, vehicle_mask):
        """
        Detect if a vehicle is in front of the ego vehicle.-6.3
        """
        height, width = vehicle_mask.shape
        roi_top = int(height * 0.65)
        roi_bottom = height
        roi_left = int(width * 0.4)
        roi_right = int(width * 0.6)
        vehicle_roi = vehicle_mask[roi_top:roi_bottom, roi_left:roi_right]
        return np.any(vehicle_roi)

    def perform_lane_change(self, left_turn):
        """
        Publish velocity commands to perform a lane change or return.
        """
        twist = Twist()
        twist.linear.x = 5.0  # Reduced speed during maneuvers
        twist.angular.z = 1.0 if left_turn else -1.0  # Turn left or right
        self.velocity_publisher.publish(twist)

    def maintain_lane(self, lane_center_offset, lin_vel):
        """
        Publish velocity commands to maintain lane.
        """
        twist = Twist()
        twist.linear.x = lin_vel  # Normal cruising speed
        twist.angular.z = -lane_center_offset  # Adjust steering to stay centered
        self.velocity_publisher.publish(twist)

    def straigthen_lane(self, lin_vel, r_l):
        """
        Publish velocity commands to keep on left lane.
        """
        twist = Twist()
        twist.linear.x = lin_vel  # Normal cruising speed
        twist.angular.z = r_l  # Adjust steering to stay centered
        self.velocity_publisher.publish(twist)


def main(args=None):
   rclpy.init(args=args)
   node = SemanticCameraNode()
   rclpy.spin(node)
   rclpy.shutdown()


if __name__ == '__main__':
   main()

