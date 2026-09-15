from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('durian_inspection_pkg')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description_content = doc.toxml()

    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")

    extra_models = [ "/usr/share/gazebo-11/models",os.path.expanduser("~/.gazebo/models")]

    for p in extra_models:
        if p not in gazebo_model_path:
            gazebo_model_path += ":" + p if gazebo_model_path else p

    os.environ["GAZEBO_MODEL_PATH"] = gazebo_model_path

    world_file = os.path.expanduser('~/durian_ws/src/durian_inspection_pkg/worlds/durian_farm.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'durian_bot', '-topic', '/robot_description', '-z','0.10'],
        output='screen'
    )
   
    return LaunchDescription([
        gazebo,

         Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': robot_description_content,'use_sim_time': True}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True
            }],
        ),


        TimerAction(
            period=8.0, 
            actions=[spawn_entity]
        ),

        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='laser_tf_pub',
                    arguments=['0.12', '0', '0.3', '0', '0', '0', 'base_link', 'laser']
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_tf_pub',
                    arguments=['0.2', '0', '0.16', '0', '0', '0', 'base_link', 'camera_link'] 
                ),
                

                Node(
                    package='depth_image_proc',
                    executable='point_cloud_xyz_node',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('image_rect', '/camera/depth/image_raw'),
                        ('camera_info', '/camera/depth/camera_info'),
                        ('points','/camera/points')
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        'approximate_sync': True
                        
                    }]
                ),

                Node(
                    package='rtabmap_sync',
                    executable='rgbd_sync',
                    name='rgbd_sync',
                    remappings=[
                        ('rgb/image', '/camera/image_raw'),
                        ('depth/image', '/camera/depth/image_raw'),
                        ('rgb/camera_info', '/camera/camera_info'),
                        ('rgbd_image', '/rtabmap/rgbd_image')
                    ],
                    parameters=[{
                        'approx_sync': True,
                        'approx_sync_max_interval': 0.1,
                        'sync_queue_size': 100,                
                        'topic_queue_size': 100,
                    }]
                ),

                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    parameters=[{
                        'use_sim_time': True,
                        'frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'map_frame_id': 'map',
                        'subscribe_depth': False,
                        'subscribe_rgb': False,
                        'subscribe_rgbd': True,     
                        'subscribe_scan': True,      
                        'Grid/Sensor': '1',      
                        'Grid/3D': 'true',       
                        'Mem/IncrementalMemory': 'true',
                        'Mem/InitWMWithAllNodes': 'false', 
                        'Cloud/VoxelSize': '0.01', 
                        'Cloud/Decimation': '1',
                     }],
                    remappings=[
                        ('scan', '/scan_filtered'),
                        ('odom', '/odom'),
                        ('rgbd_image', '/rtabmap/rgbd_image'),
                    ],
                    output='screen'
                ),
              
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

    ])