#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OSDaR23 -> OpenVINS player (mono camera + IMU).

Publishes:
  /imu0            sensor_msgs/Imu   (rad/s, m/s^2)
  /cam0/image_raw  sensor_msgs/Image (mono8, downscaled)

OSDaR23 corrimu facts (verified 2026-11-24):
  - one CSV per 10 Hz multisensor frame, one sample per file (10 Hz)
  - CORRIMUS fields are accumulated increments: gyro [rad/sample],
    accelerations [m/s/sample], corrected for gravity and earth rotation
  - vehicle/sensor frame: X = lateral (right), Y = longitudinal (forward),
    Z = vertical (up); right-handed
So: omega = gyro / dt, a_lin = acc / dt, and specific force is obtained by
re-adding gravity along +Z (the dataset ships gravity-compensated values).
"""
import glob
import os
import sys
import csv

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, Imu

DT = 0.1          # file cadence (10 Hz)
DT_INC = 0.01     # CORRIMUS increment interval (underlying 100 Hz IMU)
G = 9.80665


def load_imu(path):
    rows = []
    for f in sorted(glob.glob(os.path.join(path, "*.csv"))):
        with open(f) as fh:
            lines = list(csv.reader(fh))
        if len(lines) < 2:
            continue
        r = [float(x) for x in lines[1]]
        rows.append(r)
    return rows


def load_files(path, ext):
    return sorted(glob.glob(os.path.join(path, "*" + ext)))


def main():
    rospy.init_node("osdar_vio_player", anonymous=True)
    seq = rospy.get_param("~seq")
    img_name = rospy.get_param("~image_dir", "rgb_highres_center")
    scale = float(rospy.get_param("~scale", 0.25))
    speed = float(rospy.get_param("~speed", 1.0))
    g_add = float(rospy.get_param("~gravity", G))

    imu_dir = os.path.join(seq, "novatel_oem7_corrimu")
    img_dir = os.path.join(seq, img_name)
    imus = load_imu(imu_dir)
    imgs = load_files(img_dir, ".png")
    rospy.logwarn("player: %d imu, %d images, scale %.2f, speed %.2f", len(imus), len(imgs), scale, speed)
    if not imus or not imgs:
        rospy.logerr("missing imu or images")
        return 1

    pub_imu = rospy.Publisher("/imu0", Imu, queue_size=500)
    pub_img = rospy.Publisher("/cam0/image_raw", Image, queue_size=20)
    # wait for subscribers (OpenVINS)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (pub_img.get_num_connections() == 0 or pub_imu.get_num_connections() == 0):
        if (rospy.Time.now() - t0).to_sec() > 60:
            rospy.logerr("no subscriber after 60 s")
            return 1
        rospy.sleep(0.2)
    rospy.sleep(1.0)

    n = min(len(imus), len(imgs))
    for i in range(n):
        r = imus[i]
        ts = r[1]
        # CORRIMUS gives one increment per 0.1 s.  OpenVINS' propagator needs
        # >= 2 IMU readings between two camera times, so hold each 10 Hz rate
        # over ten 100 Hz sub-samples covering (ts-0.1, ts].  Information content
        # is unchanged (zero-order hold).
        for k in range(10):
            tsub = ts - (9 - k) * 0.01
            msg = Imu()
            msg.header.stamp = rospy.Time.from_sec(tsub)
            msg.header.frame_id = "imu"
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x = r[4] / DT_INC
            msg.angular_velocity.y = r[5] / DT_INC
            msg.angular_velocity.z = r[6] / DT_INC
            msg.linear_acceleration.x = r[7] / DT_INC
            msg.linear_acceleration.y = r[8] / DT_INC
            msg.linear_acceleration.z = r[9] / DT_INC + g_add
            pub_imu.publish(msg)
        # give the subscriber a chance to dispatch the IMU batch before the
        # image callback runs (separate ROS queues, otherwise the filter sees
        # an empty IMU interval)
        rospy.sleep(0.06)

        img = cv2.imread(imgs[i], cv2.IMREAD_GRAYSCALE)
        if img is None:
            rospy.logwarn("cannot read %s", imgs[i])
            continue
        if scale != 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        im = Image()
        im.header.stamp = rospy.Time.from_sec(ts)
        im.header.frame_id = "cam0"
        im.height, im.width = img.shape[:2]
        im.encoding = "mono8"
        im.step = int(img.strides[0])
        im.data = img.tobytes()
        pub_img.publish(im)

        if i % 10 == 0:
            rospy.logwarn("frame %d/%d t=%.3f", i, n, ts)
        rospy.sleep(max(0.0, DT / max(speed, 1e-3)))

    rospy.logwarn("player done, waiting 20 s for filter to drain")
    rospy.sleep(20.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
