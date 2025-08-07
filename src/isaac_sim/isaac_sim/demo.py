import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion
from custom_interfaces.srv import AddThreeInts

class PlanningService(Node):
    def __init__(self):
        super().__init__("add_three_ints_client")
        self.client = self.create_client(
            AddThreeInts, "planning_service"
        )  # Service name must match the server's
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the service to become available...")

        self.request = AddThreeInts.Request()  # Create a request object

    def send_request(self):
        # Fill in the request data
        point = Point()
        point.x = 0.3
        point.y = 0.4
        point.z = 0.75
        self.request.position = point
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0
        self.request.quaternion = quat
        self.request.joint_state = JointState()

        # Send the request asynchronously
        future = self.client.call_async(self.request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: {response.trajectory}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

rclpy.init()  # Initialize the ROS 2 Python client library
node = PlanningService()
node.send_request()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()