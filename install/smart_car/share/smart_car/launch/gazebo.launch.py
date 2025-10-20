from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("smart_car")

    # Robot description from Xacro
    xacro_path = os.path.join(pkg_share, "urdf", "smartcar.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro", " ", xacro_path]), value_type=str)

    # World file path
    world_path = os.path.join(pkg_share, "world", "smalltown.world")

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Spawn the robot in Gazebo
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "smartcar", "-topic", "robot_description"],
        output="screen",
    )

    # Wheel odometry node (2.2.4)
    odom_node = Node(
        package="smart_car",
        executable="wheel_odom",
        name="wheel_odom",
        output="screen",
        parameters=[{
            "wheel_radius": 0.032,
            "wheelbase": 0.257,
            "left_wheel_joint":  "front_left_wheel_joint",
            "right_wheel_joint": "front_right_wheel_joint",
            "left_steer_joint":  "front_left_wheel_steer_joint",
            "right_steer_joint": "front_right_wheel_steer_joint",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "use_sim_time": True,
        }],
        remappings=[
            ("/smart_car/vehicle_status", "/smartcar/vehicle_status"),  # <-- topic name fix
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn,
        odom_node
    ])

