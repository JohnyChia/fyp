import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # 路径配置
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    slam_config = os.path.join(pkg_path, 'config', 'slam.yaml')

    config = os.path.join(
        get_package_share_directory('durian_inspection_pkg'),
        'config',
        'robot_params.yaml' # 你存放参数的文件名
    )
    
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    return LaunchDescription([
        # 1. 基础驱动：URDF发布 (确保TF树中 base_link 到 laser 的连接由URDF定义，无需static_transform_publisher)
        # 1. 基础驱动：URDF发布 (修复版)
        # 1. 修复版：确保它能读取到 robot_description
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description_content, # 必须传入这个变量
                'use_sim_time': True
            }]
        ),
        # 2. 确保 robot_state_publisher 也正常运行
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content, 
                'use_sim_time': True
            }]
        ),
        # Node(package='durian_inspection_pkg', executable='lidar_scan', name='lidar_scanner'),

        # 2. SLAM 节点：Async 模式，实时生成 map
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

        # 3. 修正：SLAM 模式下的导航 (禁用 AMCL 和 Map Server)
                # 3. 修正：SLAM 模式下的导航 (明确禁用静态地图和定位模块)
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')), # 注意：建议改用 bringup_launch.py
        #             launch_arguments={
        #                 'params_file': nav2_params,
        #                 'use_sim_time': 'true',
        #                 'map': '',              # 关键：显式设置 map 为空
        #                 'use_lifecycle_mgr': 'true',
        #                 'autostart': 'true',
        #                 'use_slam': 'true',     # 关键：告诉 Nav2 使用 SLAM 而非 AMCL
        #             }.items()
        #         )
        #     ]
        # ),

        # 4. 业务节点 (确保不要在其他地方重复定义)
        TimerAction(
            period=4.0,
            actions=[
                Node(package='durian_inspection_pkg', executable='vision_node', name='vision_node'),
                Node(package='durian_inspection_pkg', executable='video_publisher', name='video_pub'),
                # Node(package='durian_inspection_pkg', executable='scanner_node', name='scanner_node'),
                Node(package='durian_inspection_pkg', executable='nav_controller', name='robot_controller',parameters=[config]),
                # Node(package='durian_inspection_pkg', executable='tree_visualizer', name='tree_visualizer')
            ]
        ),
        # 5. RViz2
        TimerAction(
            period=6.0,
            actions=[
                Node(package='rviz2', executable='rviz2')
            ]
        ),

        
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     parameters=[{
        #         'use_sim_time': True,
        #         'odom_frame': 'odom',
        #         'base_link_frame': 'base_link',
        #         'world_frame': 'odom', # 保持 odom，但要确保 slam_toolbox 不要覆盖它
        #         'publish_tf': False,    # 开启
        #     }],
        #     # 关键：确保 EKF 的输出被发布，但不要干扰 SLAM 对 TF 的控制
        # )
    ])