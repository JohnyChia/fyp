from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')
    config = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_path, 'maps', 'my_orchard_map.yaml')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    
    robot_description = Command(['xacro ', urdf_file])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'verbose': 'true', 
                'pause': 'false',
                'use_sim_time': use_sim_time,
                'world': '/home/johny/durian_ws/src/durian_inspection_pkg/worlds/my_world.world'
            }.items()
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time, 'publish_frequency': 30.0 }]
                ),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'durian_bot', '-file', urdf_file, '-z', '0.05'],
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time, 'autostart': True, 
                         'node_names': ['map_server', 'amcl', 'planner_server', 
                                        'controller_server', 'behavior_server', 'bt_navigator']}
                    ]
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    parameters=[config, {'use_sim_time': use_sim_time}]
                ),
                Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[config, {'use_sim_time': use_sim_time}]),
                Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[config, {'use_sim_time': use_sim_time}]),
                Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', parameters=[config, {'use_sim_time': use_sim_time}]),
                Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[config, {'use_sim_time': use_sim_time}, 
                    {'bt_xml_filename': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'}])
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='durian_inspection_pkg',
                    executable='vision_node',
                    name='vision_node',
                    prefix='taskset -c 2,3',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
            ]
        ),
    ])