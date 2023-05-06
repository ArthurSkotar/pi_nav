import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
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
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            arguments=['--orientation_stddev', '0.2', '--world_frame', 'enu', '--use_mag', 'true',
                       '--use_magnetic_field_msg', 'true', '--gain', '0.5']
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    ])
