from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("smart_car")
    xacro_path = os.path.join(pkg_share, "urdf", "smartcar.urdf.xacro")

    print("[smart_car] Using Xacro:", xacro_path)

    # NOTE: Command concatenates tokens exactly; include a space between "xacro" and the path
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
    ])

