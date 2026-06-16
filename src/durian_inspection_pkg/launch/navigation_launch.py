from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('durian_inspection_pkg')

    config = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_path, 'maps', 'my_map.yaml')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    robot_description = Command(['xacro ', urdf_file])
    nodes_to_manage = ['map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator', 'behavior_server']

    return LaunchDescription([
        # 1. 补齐 map_server 节点定义 (至关重要)
        

        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. 启动机器人模型加载 (spawn_entity)
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', 'durian_bot', '-file', urdf_file], # 确保 urdf_file 路径正确
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True, 
                'robot_description': robot_description,
                'publish_frequency': 20.0  # 确保这里读取的是你的 urdf 字符串
            }]
        ),

        # 4. 生命周期管理
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager', 
                    executable='lifecycle_manager', 
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True, 
                        'autostart': True, 
                        'node_names': nodes_to_manage
                    }]
                )
            ]
        ),

        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]
        ),

        # 2. AMCL 定位
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
             parameters=[config, {'use_sim_time': True}]),

        # 3. 核心导航控制器
        Node(package='nav2_controller', executable='controller_server', name='controller_server', 
             parameters=[config, {'use_sim_time': True}],),
        
        Node(package='nav2_planner', executable='planner_server', name='planner_server', 
             parameters=[config, {'use_sim_time': True}]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
             
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'bt_loop_duration': 500, # 增大到 500ms，减轻对 CPU 的实时性依赖
                'default_server_timeout': 20000,
                'bt_xml_filename': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml',
                'enable_groot_monitoring': False 
            }]
        ),
        # ---------------- your nodes (after NAV2 ready) ----------------
        TimerAction(
            period=1.0,
            actions=[

                Node(
                    package='durian_inspection_pkg',
                    executable='vision_node',
                    name='vision_node',
                    parameters=[{'use_sim_time': True}]
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='video_publisher',
                    name='video_pub'
                ),

            ]
        )
    ])