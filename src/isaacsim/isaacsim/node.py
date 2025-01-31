# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # Can also run as headless

from omni.isaac.core.utils.extensions import enable_extension

# Enable necessary extensions
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.services.streamclient.webrtc")

simulation_app.update()

# from omni.services.streamclient.webrtc import WebRTCServer

simulation_app.set_setting("/app/window/drawMouse", True)
simulation_app.set_setting("/app/livestream/proto", "ws")
simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
simulation_app.set_setting("/ngx/enabled", False)
# simulation_app.set_setting("/exts/omni.services.transport.server.http/port", 8211)
# simulation_app.set_setting("/app/livestream/port", 49100)

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.franka import Franka
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.objects import VisualSphere

# from omni.kit.input import get_input

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion
from custom_interfaces.srv import AddThreeInts
from collections import deque

# import keyboard

# Initialize the world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add the Panda robot
franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))

# Add a dynamic cuboid
world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0.3, 0.3, 0.3]),
        scale=np.array([0.0515, 0.0515, 0.0515]),
        color=np.array([0, 0, 1.0]),
    )
)

marker = world.scene.add(
    VisualSphere(
        prim_path="/World/moving_marker",
        name="moving_marker",
        position=np.array([0.0, 0.0, 0.5]),  # Initial position
        radius=0.5,
        color=np.array([1.0, 0.0, 0.0]),
    )
)

# Reset the world
world.reset()
action_queue = deque()


def get_socket_info():
    import socket

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return f"http://{ip}:8211/streaming/webrtc-demo/?server={ip}"


def on_keyboard_event(event):
    if event.type == "KEY_DOWN":  # Detect a key press
        if event.key == "s":  # Replace 'K' with your desired key
            print("Key 's' pressed. Triggering action...")
            node.send_request()


# Class for a ROS 2 service node
class PlanningService(Node):
    def __init__(self, robot):
        super().__init__("add_three_ints_client")
        self.client = self.create_client(
            AddThreeInts, "planning_service"
        )  # Service name must match the server's
        self.get_logger().info(get_socket_info())
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the service to become available...")

        self.robot = robot
        self.request = AddThreeInts.Request()  # Create a request object

    def send_request(self):
        # Fill in the request data
        point = Point()
        point.x = 0.3
        point.y = 0.3
        point.z = 0.3
        marker.set_world_pose(np.array([point.x, point.y, point.z]))
        self.request.position = point

        # Define Euler angles in radians (roll, pitch, yaw)
        roll = 0.0  # Rotation about X-axis
        pitch = 3.14  # Rotation about Y-axis
        yaw = 0.0  # Rotation about Z-axis (90 degrees)

        # Convert Euler angles to quaternion
        quat_conv = euler_angles_to_quat([roll, pitch, yaw])
        quat_conv = quat_conv.tolist()
        # self.get_logger().info(quat_conv)

        quat = Quaternion()
        quat.x = quat_conv[0]
        quat.y = quat_conv[1]
        quat.z = quat_conv[2]
        quat.w = quat_conv[3]
        self.request.quaternion = quat

        joint_state = JointState()
        joint_state.name = self.robot.dof_names[:7]
        joint_state.position = self.robot.get_joint_positions()[:7].tolist()
        self.request.joint_state = joint_state

        # Send the request asynchronously
        future = self.client.call_async(self.request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            for point in response.trajectory.points:
                action_queue.append(point.positions)
            # self.get_logger().info(action_queue)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


# Initialize ROS 2 and create the node
rclpy.init()  # Initialize the ROS 2 Python client library
node = PlanningService(franka)
node.send_request()

# Subscribe to keyboard events

# Simulation loop
# franka.get_articulation_controller().set_control_mode("position")
for i in range(50000):
    # if i > 1000:
    #     franka.get_articulation_controller().apply_action(
    #         ArticulationAction(
    #             joint_positions=np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 0.0, 0.0])
    #         )
    #     )
    # if keyboard.is_key_pressed("s"):
    #     node.send_request()

    if len(action_queue) > 0:
        # action = action_queue
        # node.get_logger().info(len(action_queue))
        action = action_queue.popleft()
        # node.get_logger().info(f"Action: {action_queue}")
        franka.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=action, joint_indices=range(7)),
        )
        # break
    rclpy.spin_once(node, timeout_sec=0.0)
    world.step(render=True)  # Step physics and rendering

node.destroy_node()
rclpy.shutdown()
# Close the simulation app
simulation_app.close()
