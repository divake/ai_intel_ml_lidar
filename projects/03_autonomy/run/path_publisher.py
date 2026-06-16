#!/usr/bin/python3
"""Step 1 (live): publish the taught route as a latched nav_msgs/Path in the map frame,
for Nav2 Regulated Pure Pursuit to follow. Reuses teach_path_fastlio.csv (x y yaw), which is
in the odom_lio/map frame our localization (A) provides.

  /usr/bin/python3 path_publisher.py [--path .../teach_path_fastlio.csv] [--frame map]
"""
import math, argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

DEF = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/teach_path_fastlio.csv"


class PathPub(Node):
    def __init__(self, csv, frame):
        super().__init__("taught_path_publisher")
        d = np.loadtxt(csv)
        qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # latched
        self.pub = self.create_publisher(Path, "/plan", qos)
        msg = Path(); msg.header.frame_id = frame; msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in d:
            ps = PoseStamped(); ps.header.frame_id = frame
            ps.pose.position.x = float(x); ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(yaw/2); ps.pose.orientation.w = math.cos(yaw/2)
            msg.poses.append(ps)
        self.msg = msg
        self.create_timer(1.0, self._pub)          # republish (latched, but keep stamp fresh on connect)
        self.get_logger().info(f"taught path: {len(d)} poses, frame '{frame}' -> /plan (latched)")

    def _pub(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default=DEF)
    ap.add_argument("--frame", default="map")
    a = ap.parse_args()
    rclpy.init(); n = PathPub(a.path, a.frame)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
