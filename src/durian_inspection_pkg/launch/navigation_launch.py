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
            'use_sim_time': 'true',       
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        gazebo,
    
        Node(package='robot_state_publisher', executable='robot_state_publisher', 
             parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
        ),

        Node(package='gazebo_ros', executable='spawn_entity.py', 
             arguments=['-entity', 'durian_bot', '-topic', '/robot_description', '-z', '0.2']),

        TimerAction(
            period=15.0, 
            actions=[
               TimerAction(
                    period=8.0,
                    actions=[nav2_launch]
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='map_publisher',
                    name='map_publisher',
                    parameters=[{'use_sim_time': True}], 
                    output='screen'
                ),

                Node(
                    package='depth_image_proc',
                    executable='point_cloud_xyz_node',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('image_rect', '/camera/depth/image_raw'),
                        ('camera_info', '/camera/depth/camera_info'),
                        ('points', '/camera/points')
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        'queue_size': 100,
                        'approx_sync': True
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
                        'queue_size': 50,     
                        'slop': 0.5,          
                        'use_sim_time': True  
                    }]
                ),
                
               Node(
                    package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                    parameters=[{
                        'use_sim_time': True, 
                        'subscribe_rgbd': True, 
                        'subscribe_scan': False, 
                        'subscribe_depth': False,
                        'subscribe_rgb': False,  
                        'publish_tf': True,          
                        'odom_frame_id': 'odom',
                        'map_frame_id': 'map',
                        'frame_id': 'base_footprint',
                        'tf_delay': 0.05,
                        'Rtabmap/DetectionRate': '1.0',    
                        'Grid/FromDepth': 'true',            
                        'Grid/RangeMax': '5.0',            
                        'Grid/CellSize': '0.05'            
                    }],
                    remappings=[
                        ('rgbd_image', '/rtabmap/rgbd_image'),
                        ('odom', '/odom'),
                        ('grid_map', '/map'),             
                        ('cloud_map', '/rtabmap/cloud_map')
                    ],
                    arguments=['-d'] 
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='inspection_server',
                    name='inspection_server',
                    parameters=[{'use_sim_time': True}], 
                    output='screen'
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='gui',
                    name='gui',
                    parameters=[{'use_sim_time': True}], 
                    output='screen'
                ),
            ]
        ),

        # RViz2
        Node(package='rviz2', executable='rviz2', name='rviz2', parameters=[{'use_sim_time': True}])
    ])