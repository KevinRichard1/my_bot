import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Set the environment variable to force software rendering
    set_libgl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    package_name = 'my_bot'

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output='screen')

    # Add a delay to ensure Gazebo is ready before spawning the entity
    delay = TimerAction(period=8.0, actions=[spawn_entity])  # Increased delay time

    # Return the LaunchDescription with the set environment variable included
    return LaunchDescription([
        set_libgl,  # Set the environment variable for software rendering
        rsp,
        gazebo,
        delay,  # Delay spawn_entity to give Gazebo time to initialize
    ])