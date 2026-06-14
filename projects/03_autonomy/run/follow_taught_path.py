#!/usr/bin/env python3
"""Teach-and-repeat mission executor (project 03, Phase C).
Loads the taught path (CSV: x y yaw, in the localization frame), builds a
nav_msgs/Path, and sends it to Nav2's controller_server via the FollowPath action
(Regulated Pure Pursuit follows it at the configured slow/safe speeds).

Run with /usr/bin/python3 (ROS). The robot must already be localized (FAST-LIO2
running, map->odom_lio static identity) and Nav2 controller_server active.

Usage:
  ros2 run ... OR: /usr/bin/python3 follow_taught_path.py \
      --path results/teach_path_fastlio.csv --frame odom_lio
SAFETY: speeds are capped in nav2_params.yaml (0.20 straight / 0.10 turns). Keep a
hand on the e-stop (Ctrl-C here or `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"`).
"""
import sys, math, argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TeachRepeat(Node):
    def __init__(self, csv, frame):
        super().__init__("follow_taught_path")
        self.frame = frame
        self.pts = np.loadtxt(csv)            # (N,3): x y yaw
        self.client = ActionClient(self, FollowPath, "follow_path")
        self.get_logger().info(f"Loaded {len(self.pts)} waypoints in frame '{frame}'")

    def build_path(self):
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.pts:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            qx, qy, qz, qw = yaw_to_quat(float(yaw))
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            path.poses.append(ps)
        return path

    def run(self):
        self.get_logger().info("Waiting for controller_server FollowPath action...")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("FollowPath action server not available. Is Nav2 controller_server active?")
            return
        goal = FollowPath.Goal()
        goal.path = self.build_path()
        goal.controller_id = "FollowPath"
        self.get_logger().info(">>> SENDING taught path — robot will start moving (slow). Hand on e-stop! <<<")
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("Goal rejected."); return
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("Path following finished.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/teach_path_fastlio.csv")
    ap.add_argument("--frame", default="odom_lio")
    a = ap.parse_args()
    rclpy.init()
    node = TeachRepeat(a.path, a.frame)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
