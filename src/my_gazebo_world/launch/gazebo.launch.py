from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Paths to SDF files
    box_sdf_path = '/home/hafsa/ros2_ws/src/my_gazebo_world/worlds/moving_box.sdf'
    cylinder_sdf_path = '/home/hafsa/ros2_ws/src/my_gazebo_world/worlds/moving_cylinder.sdf'

    return LaunchDescription([
        # Set TurtleBot3 model environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # Launch TurtleBot3 with default world
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
                '-x', '-1.0', '-y', '2.5', '-z', '0.25'
            ],
            output='screen'
        ),

        # Spawn the moving cylinder
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'moving_cylinder',
                '-file', cylinder_sdf_path,
                '-x', '-6.0', '-y', '-2.0', '-z', '0.0'
            ],
            output='screen'
        ),

        # Launch your Python node that commands both models
        Node(
            package='my_gazebo_plugins',
            executable='moving_box_commander.py',  # NO `.py` here
            output='screen'
        ),
    ])
