from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description_content = doc.toxml()

    # Gazebo 启动
    world_file = os.path.expanduser('~/durian_ws/src/durian_inspection_pkg/worlds/durian_farm.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True', 
            'params_file': nav2_params_file, # 使用动态路径
            'autostart': 'True',
            'use_map_server': 'False',
            'use_amcl': 'False'
        }.items()
    )

    return LaunchDescription([
        gazebo,
    
        # 机器人描述
        Node(package='robot_state_publisher', executable='robot_state_publisher', 
             parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True

            }],
        ),

        nav2_launch,

        Node(package='gazebo_ros', executable='spawn_entity.py', 
             arguments=['-entity', 'durian_bot', '-topic', '/robot_description', '-z', '0.2']),

        Node(package='tf2_ros', executable='static_transform_publisher', 
                     arguments=['0.2', '0', '0.16', '0', '0', '0', 'base_link', 'camera_link']),


        # --- 第二部分：Gazebo 实时相机逻辑 ---
        TimerAction(
            period=10.0, # 等待 Gazebo 启动
            actions=[
                # --- 第一部分：数据库地图逻辑 ---
                Node(
                    package='durian_inspection_pkg',
                    executable='map_publisher',
                    name='map_publisher',
                    output='screen' # 发布 /cloud_map
                ),
                
                # 实时深度转点云
                Node(
                    package='depth_image_proc',
                    executable='point_cloud_xyz_node',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('image_raw', '/camera/depth/image_raw'), # 必须指明是 depth 图
                        ('camera_info', '/camera/depth/camera_info'),
                        ('points', '/camera/points')
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        'queue_size': 100  # 稍微调大队列，防止丢包
                    }]
                ),
                
               Node(
                    package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
                    remappings=[
                        ('rgb/image', '/camera/image_raw'),
                        ('depth/image', '/camera/depth/image_raw'),
                        ('rgb/camera_info', '/camera/camera_info'),
                        ('rgbd_image', '/rtabmap/rgbd_image')
                    ],
                    parameters=[{
                        'approx_sync': True,
                        'queue_size': 20, 
                        'slop': 0.1  # 允许 100ms 的时间误差
                    }]
                ),
                
                # 修改 RTAB-Map 启动部分
                Node(
                    package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                    parameters=[{
                        'use_sim_time': True, 
                        'subscribe_rgbd': True, 
                        'subscribe_scan': True,
                        'subscribe_depth': False, # 改为 False，因为你用了 rgbd_sync
                        'subscribe_rgb': False,   # 改为 False
                        'publish_tf': True,
                        'odom_frame_id': 'odom',
                        'map_frame_id': 'map',
                        'frame_id': 'base_footprint',
                    }],
                    remappings=[
                        ('scan', '/scan_filtered'),
                        ('rgbd_image', '/rtabmap/rgbd_image'),
                    ]
                ),

                # Node(
                #     package='durian_inspection_pkg',
                #     executable='vision_node',
                #     name='vision_node',
                #     output='screen'
                # ),

                Node(
                    package='durian_inspection_pkg',
                    executable='navigator',
                    name='navigator',
                    output='screen'
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='inspection_server',
                    name='inspection_server',
                    output='screen'
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='gui',
                    name='gui',
                    output='screen'
                ),

            ]
        ),

        # RViz2
        Node(package='rviz2', executable='rviz2', name='rviz2', parameters=[{'use_sim_time': True}])
    ])