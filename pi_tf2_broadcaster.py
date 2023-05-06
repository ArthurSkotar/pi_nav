import math
import time

from geometry_msgs.msg import TransformStamped

import numpy as np
import imufusion

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
import json
from types import SimpleNamespace
from sensor_msgs.msg import Temperature, Imu

sample_rate = 100  # Hz

CONVERSION_CONSTANT = 10 ** 9


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('pi_frame_publisher')

        self.offset = imufusion.Offset(sample_rate)
        self.ahrs = imufusion.Ahrs()

        self.ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NED,
                                                0.5,  # gain
                                                10,  # acceleration rejection
                                                0,  # magnetic rejection
                                                5 * sample_rate)  # rejection timeout
        self.last_time = 0
        self.prev_velocity = 0
        self.prev_pos = np.array([0.0, 0.0, 0.0])

        # Declare and acquire `turtlename` parameter
        self.pi_name = self.declare_parameter(
            'pi_name', 'pi').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.handle_pi_pose,
            1)
        self.start_time = time.time()
        self.subscription  # prevent unused variable warning

    def ahrs_pos_estimation(self, imu_msg):
        gyro = self.offset.update(np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z]))
        accel = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        self.get_logger().info('gyro: "%s"' % gyro)
        self.get_logger().info('accel: "%s"' % accel)
        timestamp = (imu_msg.header.stamp.sec * CONVERSION_CONSTANT + imu_msg.header.stamp.nanosec)/CONVERSION_CONSTANT
        if self.last_time == 0:
            delta_time = 0.0
            self.last_time = timestamp
        else:
            delta_time = timestamp - self.last_time
            self.last_time = timestamp
        self.get_logger().info('timestamp: "%s"' % imu_msg.header.stamp)
        self.ahrs.update_no_magnetometer(gyro, accel, 1 / 100)
        euler = self.ahrs.quaternion.to_euler()
        self.get_logger().info('euler: "%s"' % euler)

        ahrs_internal_states = self.ahrs.internal_states
        internal_states = np.array([ahrs_internal_states.acceleration_error,
                                    ahrs_internal_states.accelerometer_ignored,
                                    ahrs_internal_states.acceleration_rejection_timer])
        acceleration = 9.81 * self.ahrs.earth_acceleration
        self.get_logger().info('earth_accel: "%s"' % self.ahrs.earth_acceleration)
        self.get_logger().info('linear_accel: "%s"' % self.ahrs.linear_acceleration)
        is_moving = np.sqrt(acceleration.dot(acceleration)) > 3  # threshold = 3 m/s/s
        if not is_moving or self.last_time == 0 or (time.time() - self.start_time) < 60:
            velocity = np.array([0.0, 0.0, 0.0])
        else:
            velocity = self.prev_velocity + delta_time * acceleration
        position = self.prev_pos + delta_time * velocity
        self.prev_pos = position
        self.get_logger().info('position: "%s"' % position)
        self.get_logger().info('acceleration: "%s"' % acceleration)
        self.get_logger().info('velocity: "%s"' % velocity)
        self.get_logger().info('delta: "%s"' % delta_time)
        return position

    def handle_pi_pose(self, msg):
        # print(msg.data)
        # data = json.loads(msg.data, object_hook=lambda d: SimpleNamespace(**d))
        t = TransformStamped()
        position = self.ahrs_pos_estimation(msg)
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.pi_name

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = quaternion_from_euler(data.r, data.p, data.y)
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
