import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import ppc
import YOLO
import u_turn
import spawn
import time

class VehicleControlNode(Node):
    def __init__(self, executor):
        super().__init__('vehicle_control_node')

        # Publisher to send driving commands to the vehicle
        self.drive_publisher = self.create_publisher(
            AckermannDrive,
            '/carla/ego_vehicle/ackermann_cmd',
            10
        )

        # Subscriber to listen for the action flag
        self.subscription = self.create_subscription(
            Bool,
            '/carla/ego_vehicle/action_flag',
            self.action_callback,
            10
        )

        # self.subscription = self.create_subscription(
        #     Float64,  # Message type (speed is a Float64)
        #     '/carla/ego_vehicle/speedometer',  # Topic to subscribe to
        #     self.speed_callback,  # Callback function to handle the received messages
        #     10  # Queue size
        # )

        self.create_subscription(
            Float32,
            '/carla/ego_vehicle/speedometer',
            self.speed_callback,
            10)

        self.action_flag = False  # Flag to track if the vehicle should stop
        self.timer = self.create_timer(0.1, self.drive)

        self.sub_control = None
        self.sub_detection = None

        self.current_fase = 1
        self.executor=executor

        self.nodes_to_destroy=[]
        self.speed=10000
        
    def phase_1(self):
        if self.sub_detection is None:
            # Start the detection node in the executor
            self.sub_detection = YOLO.YoloDetectionNode()
            self.executor.add_node(self.sub_detection)

        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = ppc.PurePursuitController('waypoints_phase1_expected.csv')
            self.executor.add_node(self.sub_control)

    def phase_2(self):
        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = u_turn.CustomControlNode()
            self.executor.add_node(self.sub_control)

    def drive(self):
        """Choose which control and detection node to use"""
        if not self.action_flag:
            if self.current_fase == 1:
                self.phase_1()
            elif self.current_fase == 2:
                self.destroy_nodes()
                self.phase_2()
        else:
            if self.sub_detection is not None:
                self.executor.remove_node(self.sub_detection)
                #self.sub_detection.destroy_node()
                self.nodes_to_destroy.append(self.sub_detection)
                self.sub_detection = None
                
            if self.sub_control is not None:
                self.executor.remove_node(self.sub_control)
                #self.sub_control.destroy_node()
                self.nodes_to_destroy.append(self.sub_control)
                self.sub_control = None



            
            if self.current_fase == 1:
                self.brake()
                if self.speed<1e-4:
                    self.current_fase += 1
                    self.action_flag = False

            else:
                self.current_fase += 1
                self.action_flag = False

    def speed_callback(self, msg):
        """Callback function that checks if the vehicle speed is zero (stopped)."""
        self.speed = msg.data  # Speed is expected to be a Float64 value

    def action_callback(self, msg):
        """Callback function to handle action flag messages."""
        if msg.data:
            self.action_flag = True  # Set stop flag to True if stop signal is received

    def brake(self):
        """Stop the vehicle by setting the speed to zero."""
        drive_msg = AckermannDrive()
        drive_msg.speed = 0.0
        self.drive_publisher.publish(drive_msg)
        self.get_logger().info('Braking...')

    def destroy_nodes(self):
        for node in self.nodes_to_destroy:
            node.destroy_node()       
        self.nodes_to_destroy=[]    

    def __del__(self):
        # Properly destroy nodes when the object is deleted
        if self.sub_detection is not None:
            self.sub_detection.destroy_node()

        if self.sub_control is not None:
            self.sub_control.destroy_node()

def main(args=None):
    spawn.main()

    rclpy.init(args=args)  # Initialize the ROS2 Python library
    executor = rclpy.executors.MultiThreadedExecutor()
    node = VehicleControlNode(executor)  # Create an instance of the VehicleControlNode

    # Create the MultiThreadedExecutor to handle multiple nodes concurrently
    
    executor.add_node(node)

    try:
        executor.spin()  # Spin the executor to handle multiple nodes concurrently
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the node is properly destroyed and shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()  # Execute the main function when the script is run
