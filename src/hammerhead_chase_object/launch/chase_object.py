from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    node1 = Node(
        package='hammerhead_chase_object',
        executable='detect_object',
        name='detect_object',
        output='screen'
    )

    node2 = Node(
        package='hammerhead_chase_object',
        executable='get_object_range',
        name='get_object_range',
        output='screen'
    )

    node3 = Node(
        package='hammerhead_chase_object',
        executable='chase_object',
        name='chase_object',
        output='screen'
    )

    try:
        # --- include: ros2 launch turtlebot3_bringup camera.robot.launch.py ---
        tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_bringup_dir, 'launch', 'camera_robot.launch.py')
            ),
            # Optional: pass launch args if that launch file expects them
            # launch_arguments={
            #     'use_sim_time': 'false',
            # }.items(),
        )
    except:
        print("Didn't launch turtle_bringup...")


    return LaunchDescription([
        camera_launch,
        node1,
        node2,
        node3
    ])
