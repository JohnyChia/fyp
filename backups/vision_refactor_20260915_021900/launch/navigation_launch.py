from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Ensure GAZEBO_RESOURCE_PATH is set for the ROS2 launch environment
    materials_path = os.path.join(pkg_path, 'config', 'materials')
    os.environ['GAZEBO_RESOURCE_PATH'] = os.path.join(materials_path, 'textures') + ':' + os.path.join(materials_path, 'scripts') + ':' + materials_path + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description_content = doc.toxml()

    world_file = os.path.expanduser('/home/johny/durian_ws/experiment/durian_farm_scale_1x.world')
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
            'autostart': 'true',
            'use_composition': 'False'
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

        TimerAction(
            period=10.0,
            actions=[
                Node(package='gazebo_ros', executable='spawn_entity.py', 
                     arguments=['-entity', 'durian_bot', '-topic', '/robot_description', '-z', '0.2', '-timeout', '120.0'])
            ]
        ),

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
                    parameters=[{'use_sim_time': True, 'world_path': world_file}],
                    output='screen'
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
                        'approx_sync_max_interval': 0.2,
                        'sync_queue_size': 100,                
                        'topic_queue_size': 100,
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
                        'sync_queue_size': 50,            
                        'topic_queue_size': 50,           
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
                    parameters=[{'use_sim_time': True, 'world_path': world_file}],
                    output='screen'
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='bridge_node',
                    name='bridge_node',
                    parameters=[{'use_sim_time': True, 'world_path': world_file}],
                    output='screen'
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='gui',
                    name='gui',
                    parameters=[{'use_sim_time': True, 'world_path': world_file}],
                    output='screen'
                ),
                
                Node(
                    package='durian_inspection_pkg',
                    executable='vision_node',
                    name='vision_node',
                    parameters=[{
                        'use_sim_time': True,
                        'model_tree_path': '/home/johny/durian_ws/models/durian_tree/best_tree_hybrid_v2.pt',
                        'model_fruit_path': '/home/johny/durian_ws/models/durian/best_v8.pt',
                        'model_leaf_path': '/home/johny/durian_ws/experiment/outputs/production_efficientnet/weights/best.pt',
                        'leaf_inference_scales': [1024, 768, 512],
                        'leaf_roi_context_scale': 1.5,
                        'leaf_roi_max_area_ratio': 0.85,
                        'leaf_roi_min_size': 80,
                    }], 
                    output='screen'
                ),
            ]
        ),

        # Node(package='rviz2', executable='rviz2', name='rviz2', parameters=[{'use_sim_time': True}])
    ])