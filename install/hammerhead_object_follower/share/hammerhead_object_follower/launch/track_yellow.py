from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    node1 = Node(
        package='hammerhead_object_follower',
        executable='find_object',
        name='find_object',
        output='screen'
    )

    node2 = Node(
        package='hammerhead_object_follower',
        executable='rotate_robot',
        name='rotate_robot',
        output='screen'
    )
    # --- include: ros2 launch turtlebot3_bringup camera.robot.launch.py ---
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'camera.robot.launch.py')
        ),
        # Optional: pass launch args if that launch file expects them
        # launch_arguments={
        #     'use_sim_time': 'false',
        # }.items(),
    )

    return LaunchDescription([
        camera_launch,
        node1,
        node2
    ])
