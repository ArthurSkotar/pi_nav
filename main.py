# -*- coding:utf-8 -*-
from . import L76X
import time
import math
from gps3 import agps3
import py_pubsub.coordTransform_py.coordTransform_utils as transform
import os

from sense_hat import SenseHat
import subprocess
import sys
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf_transformations import quaternion_about_axis


FACTOR = 1.4  # CPU Temperature adjustment factor
LINEAR_FACTOR = 2


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.sense = SenseHat()
        self.x = L76X.L76X()
        self.x.L76X_Send_Command(self.x.SET_POS_FIX_100MS)
        self.x.L76X_Set_Baudrate(115200)
        self.x.L76X_Send_Command(self.x.SET_NMEA_BAUDRATE_115200)
        self.gcj02_lng_lat = [0.0, 0.0]  # buffer to save gcj02 coordinate
        self.bd09_lng_lat = [0.0, 0.0]  # buffer to save bd09 coordinate

        # Set output message
        self.x.L76X_Send_Command(self.x.SET_NMEA_OUTPUT)

        self.x.L76X_Exit_BackupMode()
        # GPSDSocket creates a GPSD socket connection & request/retrieve GPSD output.
        self.gps_socket = agps3.GPSDSocket()
        # DataStream unpacks the streamed gpsd data into python dictionaries.
        self.data_stream = agps3.DataStream()
        self.gps_socket.connect()
        self.gps_socket.watch()

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.prev_lat = float(0.0)
        self.prev_lon = float(0.0)

    def get_temperature(self):
        current_file_dir = os.path.dirname(os.path.realpath(__file__))
        # cpu_temp_process = subprocess.Popen(["/resource/check_temp.sh"],
        #                                     stdout=subprocess.PIPE,
        #                                     stderr=subprocess.PIPE)
        # cpu_temp, stderr = cpu_temp_process.communicate()
        # cpu_temperature = float(cpu_temp)
        temperature = self.sense.get_temperature()
        calibrated_temp = temperature - LINEAR_FACTOR
        temp = round(calibrated_temp, 1)
        temp_f = round(calibrated_temp * 9 / 5 + 32, 1)
        return temp, temp_f

    def collect_imu(self):
        try:
            new_data = next(self.gps_socket)
            if new_data:
                self.data_stream.unpack(new_data)
                if self.data_stream.lat != 'n/a' and self.data_stream.lon != 'n/a':
                    self.gcj02_lng_lat = transform.wgs84_to_gcj02(float(self.data_stream.lon),
                                                                  float(
                                                                      self.data_stream.lat))
                    self.prev_lat = self.gcj02_lng_lat[0]
                    self.prev_lon = self.gcj02_lng_lat[1]
                    return self.prepare_data(self.prev_lat, self.prev_lon)
                else:
                    print('\033[1;32m Module isn\'t ready,please put the antenna outdoors and wait a moment\033[0m',
                          end='\r',
                          flush=True)
                    return {
                        "r": 0.0,
                        "p": 0.0,
                        "y": 0.0
                    }
            else:
                # print(self.prev_lat, self.prev_lon)
                return self.prepare_data(self.prev_lat, self.prev_lon)
        except Exception as e:
            print(e)
            self.x.close()

    def prepare_data(self, lon, lat):
        temp, temp_f = self.get_temperature()
        humidity = round(self.sense.get_humidity(), 1)
        orientation = self.sense.get_orientation()

        time_stamp = time.time()
        measurement = {
            "time": time_stamp,
            "altitude": self.data_stream.alt,
            "lon": lon,
            "lat": lat,
            "speed": self.data_stream.speed,
            "temp": temp,
            "humidity": humidity,
            "r": orientation['roll'],
            "p": orientation['pitch'],
            "y": orientation['yaw']
        }
        return measurement

    # exit()

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.frame_id = "world"

        # Read the acceleration vals
        accel_x = self.sense.get_accelerometer_raw()["x"]
        accel_y = self.sense.get_accelerometer_raw()["y"]
        accel_z = self.sense.get_accelerometer_raw()["z"]

        # Calculate a quaternion representing the orientation
        accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        # Read the gyro vals
        gyro_x = self.sense.get_gyroscope_raw()["x"]
        gyro_y = self.sense.get_gyroscope_raw()["y"]
        gyro_z = self.sense.get_gyroscope_raw()["z"]

        # Load up the IMU message
        o = imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing: "%s"' % imu_msg)


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


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
