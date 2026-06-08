#!/usr/bin/env python3
"""Measure RealSense D455 stream rates over ROS 2.

WHY THIS EXISTS:
  - All RealSense topics publish with best_effort QoS (SensorDataQoS). A default
    reliable subscriber (and `ros2 topic hz` in this Jazzy build) receives NOTHING
    from them -- it looks like dead hardware but is just a QoS mismatch.
  - `python3` on PATH is conda's 3.13, which can't import rclpy. Run me with the
    SYSTEM interpreter:  /usr/bin/python3 check_rates.py

USAGE:
  1) Launch the camera (one node only):
       source /opt/ros/jazzy/setup.bash
       source /home/nus-ai/ros2_ws/install/setup.bash
       ros2 launch realsense2_camera rs_launch.py \
         enable_color:=true enable_depth:=true \
         enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
         accel_fps:=200 gyro_fps:=200
  2) In another sourced shell:
       /usr/bin/python3 /home/nus-ai/divek_nus/ml_lidar/check_rates.py
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Image

TOPICS = [
    ("/camera/camera/imu", Imu),
    ("/camera/camera/accel/sample", Imu),
    ("/camera/camera/gyro/sample", Imu),
    ("/camera/camera/color/image_raw", Image),
    ("/camera/camera/depth/image_rect_raw", Image),
]
DURATION_S = 6.0


def main():
    rclpy.init()
    node = Node("rate_counter")
    counts = {t: 0 for t, _ in TOPICS}

    def make_cb(topic):
        def cb(_msg):
            counts[topic] += 1
        return cb

    for topic, msg_type in TOPICS:
        node.create_subscription(msg_type, topic, make_cb(topic), qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < DURATION_S:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.time() - t0

    print(f"--- measured over {elapsed:.1f}s (best_effort subscriber) ---")
    for topic, _ in TOPICS:
        n = counts[topic]
        print(f"{topic:45s}: {n/elapsed:7.1f} Hz  ({n} msgs)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
