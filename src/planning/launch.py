import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="panda_arm_config")
        # .robot_description(file_path="config/panda.urdf.xacro")
        # .robot_description_semantic(file_path="config/panda.srdf")
        # # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    kinematics_yaml = load_yaml(
        "panda_arm_config", "config/kinematics.yaml"
    )

    planning_yaml = load_yaml(
        "panda_arm_config", "config/ompl_planning.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="planning",
                executable="node",
                name="planning_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    kinematics_yaml,
                    planning_yaml,
                ],
            )
        ]
    )
