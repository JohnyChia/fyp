import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')

    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    slam_config = os.path.join(pkg_path, 'config', 'slam.yaml')

    
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'durian_bot', '-file', '/home/johny/durian_ws/src/durian_inspection_pkg/urdf/robot.urdf'],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description_content, 
                'use_sim_time': True
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content, 
                'use_sim_time': True
            }]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    parameters=[slam_config, {'use_sim_time': True, 'publish_tf': True}],
                )
            ]
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(package='durian_inspection_pkg', executable='vision_node', name='vision_node'),
                Node(package='durian_inspection_pkg', executable='video_publisher', name='video_pub'),
            ]
        ),
    ])