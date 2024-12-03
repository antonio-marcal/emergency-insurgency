import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import ppc
import YOLO
import u_turn
import semantic_camera
import overtake
import spawn
import destroy_actor
import highway_detector
import carla
import highway_control
import time

class VehicleControlNode(Node):
    def __init__(self, executor, world,ego_vehicle):
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

         # Subscriber to listen for the waypoints_done flag
        self.subscription = self.create_subscription(
            Bool,
            '/carla/ego_vehicle/waypoints_done',
            self.waypoints_callback,
            10
        )

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

        self.world = world
        self.ego_vehicle = ego_vehicle
        
    def phase_1(self):
        if self.sub_detection is None:
            # Start the detection node in the executor
            self.sub_detection = YOLO.YoloDetectionNode()
            self.executor.add_node(self.sub_detection)

        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = ppc.PurePursuitController('waypoints_phase1_expected.csv', 30, 40, 8.0)
            self.executor.add_node(self.sub_control)

    def phase_2(self):
        if self.sub_detection is None:
            # Start the detection node in the executor
            self.sub_detection = semantic_camera.SemanticCameraNode(dumb_mode=True)
            self.executor.add_node(self.sub_detection)
            
        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = u_turn.CustomControlNode(self.world)
            self.executor.add_node(self.sub_control)

    def phase_3(self):

        if self.sub_detection is None:
            # Start the detection node in the executor
            self.sub_detection = semantic_camera.SemanticCameraNode(world=self.world)
            self.executor.add_node(self.sub_detection)

        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = overtake.CombinedControlNode(self.world)
            self.executor.add_node(self.sub_control)

        #destroy_actor.move_slow_vehicle("*citroen.c3*", self.world)

    def phase_4(self):
        # if self.sub_detection is None:
        #     # Start the detection node in the executor
        #     self.sub_detection = YOLO.YoloDetectionNode()
        #     self.executor.add_node(self.sub_detection)

        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = ppc.PurePursuitController('waypoints_phase4.csv', 70, 15, 4.0)
            self.executor.add_node(self.sub_control)

    def phase_5(self):
        destroy_actor.destroy_actor('*pedestrian.0036*', self.world)


        if self.sub_detection is None:
            # Start the detection node in the executor
            self.sub_detection = highway_detector.HighwayDetectionNode(self.world)
            self.executor.add_node(self.sub_detection)

        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = ppc.PurePursuitController('waypoints_phase45.csv', 70, 15, 4.0)
            self.executor.add_node(self.sub_control)

    def phase_6(self):
        if self.sub_detection is None:
            # Start the detection node in the executor
            self.sub_detection = semantic_camera.SemanticCameraNode(dumb_mode=True)
            self.executor.add_node(self.sub_detection)

        if self.sub_control is None:
            # Start the control node in the executor
            self.sub_control = highway_control.CombinedControlNode(self.world)
            self.executor.add_node(self.sub_control)

    def phase_7(self):
        specific_walker_type = 'walker.pedestrian.0036'  # Victim
        spawn_location = (-40.2, 144.1, 0.1)  # Example location (x, y, z)
        spawn_rotation = (90, 0, 0)  # Example rotation (pitch, yaw, roll)
        spawn.spawn_specific_vehicle(self.world, self.world.get_blueprint_library(),
                                     specific_walker_type, spawn_location, spawn_rotation)
        self.action_flag = True


    def drive(self):
        """Choose which control and detection node to use"""
        if not self.action_flag:
            if self.current_fase == 1:
                self.phase_1()
            elif self.current_fase == 2:
                self.destroy_nodes()
                self.phase_2()
            elif self.current_fase == 3:
                self.destroy_nodes()
                self.phase_3()
            elif self.current_fase == 4:
                self.destroy_nodes()
                self.phase_4()
            elif self.current_fase == 5:
                self.destroy_nodes()
                time.sleep(5)
                self.phase_5()
            elif self.current_fase == 6:
                self.destroy_nodes()
                self.phase_6()
            elif self.current_fase == 7:
                self.destroy_nodes()
                self.phase_7()
            else: 
                self.destroy_nodes()

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

            
            if self.current_fase in (1, 4, 6):
                self.brake()
                if self.speed<1e-4:
                    self.current_fase += 1
                    self.action_flag = False
                    self.ego_vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Special1 | carla.VehicleLightState.LowBeam | carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker))

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

    def waypoints_callback(self, msg):
        """Callback function to handle waypoints flag messages."""
        if msg.data and self.current_fase == 4:
            self.action_flag = True

    def brake(self):
        self.ego_vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Special1 | carla.VehicleLightState.LowBeam | carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Brake))
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
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(carla.Location(x=-7,y=35,z=155),
                                            carla.Rotation(pitch=-90, yaw=-90)))

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_hybrid_physics_mode(False)

    spawn.main(world)

    for actor in world.get_actors().filter('vehicle.*'):
        if 'ego_vehicle' in actor.attributes['role_name']:  # Check if the actor is the ego vehicle
            ego_vehicle = actor
            break  # Stop once we find the ego vehicle

    if ego_vehicle is None:
        print("Ego vehicle not found!")

    # Turn on the headlights
    ego_vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Special1 | carla.VehicleLightState.LowBeam | carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker))

    rclpy.init(args=args)  # Initialize the ROS2 Python library
    executor = rclpy.executors.MultiThreadedExecutor()
    node = VehicleControlNode(executor, world,ego_vehicle)  # Create an instance of the VehicleControlNode

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
