from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    materials_path = os.path.join(pkg_path, 'config', 'materials')
    gazebo_builtin = '/usr/share/gazebo-11'
    resource_paths = [
        os.path.join(materials_path, 'textures'),
        os.path.join(materials_path, 'scripts'),
        materials_path,
        gazebo_builtin,
        os.environ.get('GAZEBO_RESOURCE_PATH', ''),
    ]
    os.environ['GAZEBO_RESOURCE_PATH'] = ':'.join(
        path for path in resource_paths if path
    )

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description_content = doc.toxml()

    world_file = os.path.expanduser('/home/johny/durian_ws/experiment/durian_farm_scale_1x.world')
    navigation_only_validation = LaunchConfiguration('navigation_only_validation')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    enable_rtabmap_diagnostics = LaunchConfiguration('enable_rtabmap_diagnostics')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': gazebo_gui,
        }.items()
    )

    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    default_leaf_deployment_config_file = os.path.join(
        pkg_path, 'config', 'leaf_detector_deployment.yaml'
    )
    leaf_deployment_config_file = LaunchConfiguration('leaf_deployment_config')
    navigation_map_file = os.path.join(
        pkg_path, 'maps', 'controlled_orchard_navigation.yaml'
    )

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
        DeclareLaunchArgument(
            'navigation_only_validation', default_value='false',
            description='Run navigation/capture-state validation without perception or DB results.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true',
            description='Start the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'enable_rtabmap_diagnostics', default_value='false',
            description=(
                'Enable non-authoritative RTAB-Map diagnostics. It never '
                'publishes navigation TF or /map.'
            ),
        ),
        DeclareLaunchArgument(
            'leaf_deployment_config',
            default_value=default_leaf_deployment_config_file,
            description=(
                'Validated backend/checkpoint deployment YAML; selects '
                'yolo26n, yolo26n_p2, rfdetr_n, or dfine_n without source edits.'
            ),
        ),
        gazebo,

        # SIMULATION ONLY: Gazebo world odometry is the authoritative global
        # localization source. RTAB-Map is forbidden from publishing map->odom.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='simulation_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

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
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    parameters=[{
                        'use_sim_time': True,
                        'yaml_filename': navigation_map_file,
                    }],
                    output='screen',
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_static_map',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['map_server'],
                    }],
                    output='screen',
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
                    }],
                    condition=IfCondition(enable_rtabmap_diagnostics),
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
                        'publish_tf': False,
                        'odom_frame_id': 'odom',
                        'map_frame_id': 'rtabmap_map',
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
                        # RTAB-Map's OccupancyGrid output is named "map".
                        # Keep it diagnostic-only and never compete with the
                        # authoritative Nav2 map_server on /map.
                        ('map', '/rtabmap/grid_map'),
                        ('cloud_map', '/rtabmap/cloud_map')
                    ],
                    arguments=['-d'],
                    condition=IfCondition(enable_rtabmap_diagnostics),
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='inspection_server',
                    name='inspection_server',
                    parameters=[{
                        'use_sim_time': True,
                        'world_path': world_file,
                        'auto_shutdown_after_completion': True,
                        'leaf_deployment_config_path': leaf_deployment_config_file,
                        'navigation_only_validation': navigation_only_validation,
                    }],
                    output='screen',
                    on_exit=Shutdown(
                        reason='inspection server completed; stopping system'
                    ),
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='bridge_node',
                    name='bridge_node',
                    parameters=[{'use_sim_time': True, 'world_path': world_file}],
                    output='screen',
                    condition=UnlessCondition(navigation_only_validation),
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='gui',
                    name='gui',
                    parameters=[{'use_sim_time': True, 'world_path': world_file}],
                    output='screen',
                    condition=UnlessCondition(navigation_only_validation),
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='vision_node',
                    name='vision_node',
                    parameters=[{
                        'use_sim_time': True,
                        'capture_root': '/home/johny/durian_ws/captured_images',
                        'database_path': '/home/johny/durian_ws/durian_inspection.db',
                        'capture_interval_sec': 0.1,
                        'rgb_depth_tolerance_sec': 0.15,
                        'tf_lookup_timeout_sec': 0.3,
                        # Capture occurs only after the stationary localization
                        # gate. Bound latest-TF fallback rather than persisting
                        # observations with missing transforms.
                        'tf_latest_fallback_max_age_sec': 1.5,
                    }],
                    output='screen',
                    condition=UnlessCondition(navigation_only_validation),
                ),

                Node(
                    package='durian_inspection_pkg',
                    executable='processing_node',
                    name='processing_node',
                    parameters=[{
                        'use_sim_time': True,
                        'database_path': '/home/johny/durian_ws/durian_inspection.db',
                        'processed_root': '/home/johny/durian_ws/processed_images',
                        'model_tree_path': '/home/johny/durian_ws/models/durian_tree/best_tree_hybrid_v2.pt',
                        'model_fruit_path': '/home/johny/durian_ws/models/durian/best_v8.pt',
                        'leaf_deployment_config_path': leaf_deployment_config_file,
                        'leaf_roi_context_scale': 1.5,
                        'leaf_roi_max_area_ratio': 0.85,
                        'leaf_roi_min_size': 80,
                        'diagnostic_logging_enabled': False,
                        'diagnostic_artifacts_enabled': False,
                    }],
                    output='screen',
                    condition=UnlessCondition(navigation_only_validation),
                ),
            ]
        ),

        TimerAction(
            # The inspection server itself rejects start until Nav2 is ready.
            # Give the delayed Gazebo spawn and Nav2 lifecycle bringup enough
            # time before this single validation-only start request.
            period=50.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/control_inspection',
                        'durian_message/srv/InspectionControl',
                        '{command: start}',
                    ],
                    output='screen',
                    condition=IfCondition(navigation_only_validation),
                )
            ],
        ),

    ])
