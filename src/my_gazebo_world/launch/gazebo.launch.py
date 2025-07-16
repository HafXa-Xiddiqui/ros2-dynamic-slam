from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_cartographer_dir = get_package_share_directory('turtlebot3_cartographer')

    # Paths to SDF files
    box_sdf_path = '/home/hafsa/ros2_ws/src/my_gazebo_world/worlds/moving_box.sdf'
    cylinder_sdf_path = '/home/hafsa/ros2_ws/src/my_gazebo_world/worlds/moving_cylinder.sdf'

    return LaunchDescription([
        # Set TurtleBot3 model environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

        # Launch TurtleBot3 with house world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_house.launch.py')
            )
        ),

        # Spawn the moving box
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'moving_box',
                '-file', box_sdf_path,
                '-x', '2.466', '-y', '0.34', '-z', '0.24'
            ],
            output='screen'
        ),

        # Spawn the moving cylinder
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'moving_cylinder',
                '-file', cylinder_sdf_path,
                '-x', '-5.14', '-y', '3.2', '-z', '0.24'
            ],
            output='screen'
        ),

        # Launch your Python node that commands both models
        Node(
            package='my_gazebo_plugins',
            executable='moving_box_commander.py',  # controls both
            output='screen'
        ),

        # Launch your Python node that commands both models
        Node(
            package='yolo_detector',
            executable='yolo_node',  # controls both
            output='screen'
        ),
        Node(
            package='scan_filter',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen'
        ),

        # ✅ Launch Cartographer for SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_cartographer_dir, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'scan_topic': '/scan_filtered'  # 👈 remap!
            }.items()                       
        ),
    ])
