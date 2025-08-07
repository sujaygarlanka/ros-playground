from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="isaac_sim",  # Name of your package
                executable="node",  # Name of your Python script or entry point
                name="isaac_sim_node",  # Node name
                output="screen",  # Print output to the console
                prefix="/home/sujay/.local/share/ov/pkg/isaac-sim-4.0.0/python.sh",
                parameters=[]
            )
        ]
    )
