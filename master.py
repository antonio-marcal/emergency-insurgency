import rclpy
import threading
import ppc
import YOLO
import u_turn
import spawn
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive  # Message type for vehicle control
from std_msgs.msg import Bool  # Message type for stop flag


class VehicleControlNode(Node):
    def __init__(self):
        super().__init__('vehicle_control_node')  # Initialize the node with a name

        # Publisher to send driving commands to the vehicle
        self.drive_publisher = self.create_publisher(
            AckermannDrive,  # Message type for vehicle control
            '/carla/ego_vehicle/ackermann_cmd',  # Topic name for vehicle commands
            10  # Queue size
        )

        # Subscriber to listen for the action flag
        self.subscription = self.create_subscription(
            Bool,
            '/carla/ego_vehicle/action_flag',  # Topic name for action flag
            self.action_callback,
            10  # Queue size
        )

        self.action_flag = False  # Flag to track if the vehicle should stop

        # Timer to send driving commands continuously every 0.1 seconds
        self.timer = self.create_timer(0.1, self.drive)

        self.sub_control = None
        self.sub_detection = None

        self.control_thread = None
        self.detection_thread = None

        self.current_fase = 1

    def phase_1(self):
        
        if self.sub_detection is None:
            # Start the detection node in a new thread
            self.sub_detection = YOLO.YoloDetectionNode()
            self.detection_thread = threading.Thread(target=rclpy.spin, args=(self.sub_detection,))
            self.detection_thread.start()

        if self.sub_control is None:
            # Start the control node in a new thread
            self.sub_control = ppc.PurePursuitController('waypoints_phase1_expected.csv')
            self.control_thread = threading.Thread(target=rclpy.spin, args=(self.sub_control,))
            self.control_thread.start()

    def phase_2(self):
        if self.sub_control is None:
            # Start the control node in a new thread
            self.sub_control = u_turn.CustomControlNode()
            self.control_thread = threading.Thread(target=rclpy.spin, args=(self.sub_control,))
            self.control_thread.start()

    def drive(self):
        """Choose which control and detection node to use"""
        if not self.action_flag:

            match self.current_fase:
                case 1:
                    self.phase_1()
                case 2:
                    self.phase_2()

        else:
            if self.sub_control is not None:
                self.sub_control.destroy_node()
                self.sub_control = None
                if self.control_thread is not None:
                    self.control_thread.join()  # Wait for the thread to finish
                    self.control_thread = None

            if self.sub_detection is not None:
                self.sub_detection.destroy_node()
                self.sub_detection = None
                if self.detection_thread is not None:
                    self.detection_thread.join()  # Wait for the thread to finish
                    self.detection_thread = None

            self.action_flag = False

            if self.current_fase == 1:
                self.brake()

            self.current_fase += 1

    def action_callback(self, msg):
        """Callback function to handle action flag messages."""
        if msg.data:
            self.action_flag = True  # Set stop flag to True if stop signal is received

    def brake(self):
        """Stop the vehicle by setting the speed to zero."""
        drive_msg = AckermannDrive()  # Create a new drive command message
        drive_msg.speed = 0.0  # Set the vehicle speed to zero
        self.drive_publisher.publish(drive_msg)  # Publish the drive command
        self.get_logger().info('Braking...')  # Log that the vehicle is braking

    def __del__(self):
        # Properly destroy nodes and join threads when the object is deleted
        if self.sub_control is not None:
            self.sub_control.destroy_node()
            self.sub_control = None
            if self.control_thread is not None:
                self.control_thread.join()
                self.control_thread = None

        if self.sub_detection is not None:
            self.sub_detection.destroy_node()
            self.sub_detection = None
            if self.detection_thread is not None:
                self.detection_thread.join()
                self.detection_thread = None


def main(args=None):
    spawn.main()

    rclpy.init(args=args)  # Initialize the ROS2 Python library
    node = VehicleControlNode()  # Create an instance of the VehicleControlNode

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

    rclpy.shutdown()  # Shutdown the ROS2 Python library


if __name__ == '__main__':
    main()  # Execute the main function when the script is run
