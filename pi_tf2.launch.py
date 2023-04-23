from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='talker',
            name='pi_talker'
        ),
        Node(
            package='py_pubsub',
            executable='pi_tf2_broadcaster',
            name='pi_broadcaster',
            parameters=[
                {'pi_name': 'pi1'}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id',
                       'world', '--child-frame-id', 'pistatic']
        ),
    ])