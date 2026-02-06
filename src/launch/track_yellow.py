from launch import LaunchDescription
from launch_ros.actions import Node


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

    return LaunchDescription([
        node1,
        node2
    ])
