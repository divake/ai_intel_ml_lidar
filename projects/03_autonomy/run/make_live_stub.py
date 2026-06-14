#!/usr/bin/env python3
"""Generate a short STRAIGHT stub path starting at the robot's CURRENT pose, going
forward along its current heading — in the odom_lio frame. Use this for the supervised
2 m creep test (safety-ladder rung 3) so the path is frame-aligned no matter where the
odom_lio origin currently sits. Run with /usr/bin/python3.

  /usr/bin/python3 make_live_stub.py --dist 2.0 --out results/stub_live_2m.csv
"""
import sys, math, argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=2.0, help="stub length (m)")
    ap.add_argument("--step", type=float, default=0.10, help="waypoint spacing (m)")
    ap.add_argument("--parent", default="odom_lio")
    ap.add_argument("--child", default="base_link")
    ap.add_argument("--out", default="/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/stub_live_2m.csv")
    a = ap.parse_args()

    rclpy.init()
    node = Node("make_live_stub")
    buf = Buffer()
    TransformListener(buf, node)
    # let TF fill
    t0 = time.time()
    tf = None
    while time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform(a.parent, a.child, rclpy.time.Time())
            break
        except Exception:
            continue
    if tf is None:
        print(f"ERROR: no {a.parent}->{a.child} transform. Is the stack up?")
        sys.exit(1)

    x0 = tf.transform.translation.x
    y0 = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    print(f"current pose: x={x0:.2f} y={y0:.2f} yaw={math.degrees(yaw):.1f} deg")

    n = max(2, int(a.dist / a.step) + 1)
    rows = []
    for i in range(n):
        d = i * a.step
        rows.append((x0 + d * math.cos(yaw), y0 + d * math.sin(yaw), yaw))
    np.savetxt(a.out, np.array(rows), fmt="%.4f")
    print(f"wrote {n} waypoints ({a.dist:.1f} m forward) -> {a.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
