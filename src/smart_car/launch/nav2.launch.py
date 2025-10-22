# smart_car/launch/nav2.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    map_yaml     = LaunchConfiguration('map')

    pkg_share = FindPackageShare('smart_car')

    # Defaults (can be overridden at launch)
    default_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    default_map    = PathJoinSubstitution([pkg_share, 'nav2_map', 'smalltown_world.yaml'])

    # Optional: built-in BT XML from Nav2 package
    nav2_bt_share = FindPackageShare('nav2_bt_navigator')
    default_bt_xml = PathJoinSubstitution(
        [nav2_bt_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',  default_value=default_params,
                              description='Nav2 parameters YAML'),
        DeclareLaunchArgument('map',          default_value=default_map,
                              description='Map YAML (includes image path)'),

        # Map server (serves /map)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml},
            ],
        ),

        # AMCL (produces map->odom)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_file
            ],
        ),

        # Controller server (local planner)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_file
            ],
        ),

        # Planner server (global planner)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_file
            ],
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'default_bt_xml_filename': default_bt_xml},
                params_file
            ],
        ),

        # Waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_file
            ],
        ),

        # Recoveries / behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_file
            ],
        ),

        # Lifecycle manager to bring everything up
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'behavior_server'
                ],
            }],
        ),
    ])

