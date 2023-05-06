#!/usr/bin/env python3
import numpy as np
import csv, datetime
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import os

try:
    from sense_hat import SenseHat
except:
    from sense_emu import SenseHat
import subprocess
import sys
import time

FACTOR = 1.4  # CPU Temperature adjustment factor
LINEAR_FACTOR = 2
sense = SenseHat()


def get_accel():
    ax = sense.get_accelerometer_raw()["x"]
    ay = sense.get_accelerometer_raw()["y"]
    az = sense.get_accelerometer_raw()["z"]
    return ax, ay, az


def accel_fit(x_input, m_x, b):
    return (m_x * x_input) + b  # fit equation for accel calibration


def accel_cal():
    print("-" * 50)
    print("Accelerometer Calibration")
    mpu_offsets = [[], [], []]  # offset array to be printed
    axis_vec = ['z', 'y', 'x']  # axis labels
    cal_directions = ["upward", "downward", "perpendicular to gravity"]  # direction for IMU cal
    cal_indices = [2, 1, 0]  # axis indices
    for qq, ax_qq in enumerate(axis_vec):
        ax_offsets = [[], [], []]
        print("-" * 50)
        for direc_ii, direc in enumerate(cal_directions):
            input("-" * 8 + " Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -" + \
                  ax_qq + "-axis pointed " + direc)
            # [mpu6050_conv() for ii in range(0,cal_size)] # clear buffer between readings
            mpu_array = []
            while len(mpu_array) < cal_size:
                try:
                    ax, ay, az = get_accel()
                    mpu_array.append([ax, ay, az])  # append to array
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:, cal_indices[qq]]  # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts, _ = curve_fit(accel_fit, np.append(np.append(ax_offsets[0],
                                                            ax_offsets[1]), ax_offsets[2]),
                             np.append(np.append(1.0 * np.ones(np.shape(ax_offsets[0])),
                                                 -1.0 * np.ones(np.shape(ax_offsets[1]))),
                                       0.0 * np.ones(np.shape(ax_offsets[2]))),
                             maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts  # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets


if __name__ == '__main__':
    accel_labels = ['a_x', 'a_y', 'a_z']  # gyro labels for plots
    cal_size = 1000  # number of points to use for calibration
    accel_coeffs = accel_cal()  # grab accel coefficients
    print(accel_coeffs)
    for i in range(200):
        data = get_accel()
        print("Uncalibrated:", data)
        calibrated = []
        for ii in range(0, 3):
            calibrated.insert(ii, accel_fit(data[ii], *accel_coeffs[ii]))
        print("Calibrated:", calibrated)
        time.sleep(3)
