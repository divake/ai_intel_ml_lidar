#!/usr/bin/python3
"""robot_drive — L0 locomotion box for the Agilex Scout Mini ("software remote").

THE ONE REUSABLE THING. Every project on this robot uses THIS to move, unchanged.
It is the software equivalent of the handheld remote: you give it intent
(forward / back / spin / arc / stop, with a speed), it makes the wheels do it
safely. It owns NOTHING about navigation, maps, or localization — just motion.

----------------------------------------------------------------------------
How the Scout actually moves (the fact that ends the "control the 4 wheels"
confusion):

  The Scout Mini is SKID-STEER (a tank). You do NOT steer it by driving 4
  wheels individually. You give just TWO numbers:

      linear.x   forward(+) / back(-)   in m/s
      angular.z  left(+)  / right(-)    in rad/s   (REP-103: +z = CCW = left)

  Agilex's ugv_sdk firmware converts those two numbers into left-side and
  right-side wheel speeds over CAN:
      forward  -> both sides equal, forward
      spin     -> one side forward, other side backward
      arc      -> sides at different speeds
  The handheld remote sends these same two numbers. So anything the remote can
  do, this class can do. There is no lower level to "unlock" — it's in firmware.
----------------------------------------------------------------------------

What this box ADDS on top of raw /cmd_vel (the "non-breakable" part):
  * speed/turn CLAMPS                — never exceeds safe caps
  * acceleration RAMPS               — smooth starts/stops, no jerk (like a real controller)
  * a DEAD-MAN watchdog              — if the commander above goes silent, decay to zero
  * a steady publish stream          — the Scout stops if /cmd_vel stalls; we keep it fed
  * a hard-stop on exit / Ctrl-C     — the robot ALWAYS stops when this process dies

Use as a LIBRARY (your algorithm streams setpoints):
    rclpy.init()
    drive = RobotDrive()
    ...your loop... drive.set(v, w)          # call repeatedly; ramps + safety applied
    drive.hard_stop(); drive.destroy_node()

Use as a CLI (prove each primitive in isolation, you watch with e-stop in hand):
    /usr/bin/python3 robot_drive.py demo            # full scripted sequence
    /usr/bin/python3 robot_drive.py forward --v 0.15 --t 3
    /usr/bin/python3 robot_drive.py left    --w 0.4  --t 3
    /usr/bin/python3 robot_drive.py stop

Run with /usr/bin/python3 (conda python is 3.13 and breaks rclpy).
Prereq for live test: the Scout base must be up (it subscribes /cmd_vel):
    ros2 launch scout_cmd scout_mini.launch.py
(For this isolated L0 test you do NOT need FAST-LIO2 or Nav2 — base only.)
"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotDrive(Node):
    """Skid-steer locomotion primitives over /cmd_vel, with ramps + a dead-man.

    Single source of truth: a fixed-rate timer publishes the CURRENT velocity,
    which ramps toward the SETPOINT each tick. You only ever set the setpoint.
    """

    def __init__(self, *, topic="/cmd_vel", rate_hz=50.0,
                 max_v=0.30, max_w=0.80,
                 accel_v=0.6, accel_w=2.0,
                 cmd_timeout=0.7):
        super().__init__("robot_drive")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.dt = 1.0 / rate_hz
        self.max_v, self.max_w = max_v, max_w          # hard speed caps (m/s, rad/s)
        self.accel_v, self.accel_w = accel_v, accel_w  # ramp rates (m/s^2, rad/s^2)
        self.cmd_timeout = cmd_timeout                 # dead-man: silence -> coast to 0

        self._tgt_v = self._tgt_w = 0.0                # setpoint (what you ask for)
        self._cur_v = self._cur_w = 0.0                # actual published (ramped)
        self._last_cmd = time.monotonic()
        self._lock = threading.Lock()
        self.timer = self.create_timer(self.dt, self._tick)

    # ---- primitives: set a velocity SETPOINT (non-blocking) -------------------
    @staticmethod
    def _clamp(x, lim):
        return max(-lim, min(lim, x))

    def set(self, v, w):
        """Set the velocity setpoint. Call repeatedly to stream (feeds the dead-man)."""
        with self._lock:
            self._tgt_v = self._clamp(float(v), self.max_v)
            self._tgt_w = self._clamp(float(w), self.max_w)
            self._last_cmd = time.monotonic()

    def forward(self, v):    self.set(abs(v), 0.0)     # straight ahead
    def back(self, v):       self.set(-abs(v), 0.0)    # straight back
    def spin_left(self, w):  self.set(0.0, abs(w))     # rotate in place, CCW
    def spin_right(self, w): self.set(0.0, -abs(w))    # rotate in place, CW
    def arc(self, v, w):     self.set(v, w)            # curve: drive + turn together
    def stop(self):          self.set(0.0, 0.0)        # ramp to a halt

    # ---- the engine: ramp current -> setpoint, publish, watch the dead-man ----
    def _ramp(self, cur, tgt, accel):
        step = accel * self.dt
        if tgt > cur: return min(cur + step, tgt)
        if tgt < cur: return max(cur - step, tgt)
        return cur

    def _tick(self):
        with self._lock:
            tgt_v, tgt_w = self._tgt_v, self._tgt_w
            if (time.monotonic() - self._last_cmd) > self.cmd_timeout:
                tgt_v = tgt_w = 0.0                     # commander went silent -> stop
        self._cur_v = self._ramp(self._cur_v, tgt_v, self.accel_v)
        self._cur_w = self._ramp(self._cur_w, tgt_w, self.accel_w)
        msg = Twist()
        msg.linear.x = self._cur_v
        msg.angular.z = self._cur_w
        self.pub.publish(msg)

    @property
    def moving(self):
        return abs(self._cur_v) > 1e-3 or abs(self._cur_w) > 1e-3

    # ---- blocking helpers for scripted / isolated tests -----------------------
    def drive_for(self, v, w, duration, *, ramp_down=True):
        """BLOCKING: hold (v, w) for `duration` s (streaming setpoints), then stop.
        Spins the node itself, so it's self-contained for CLI/scripted use."""
        end = time.monotonic() + float(duration)
        while rclpy.ok() and time.monotonic() < end:
            self.set(v, w)                              # refresh each loop (dead-man fed)
            rclpy.spin_once(self, timeout_sec=self.dt)
        if ramp_down:
            self.smooth_stop()

    def smooth_stop(self, timeout=2.0):
        """BLOCKING: ramp velocity to zero, hold zero, then hard-stop."""
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            self.set(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=self.dt)
            if not self.moving:
                break
        self.hard_stop()

    def hard_stop(self):
        """Publish zeros immediately, several times. Last-resort safety on exit."""
        with self._lock:
            self._tgt_v = self._tgt_w = 0.0
        self._cur_v = self._cur_w = 0.0
        z = Twist()
        for _ in range(5):
            self.pub.publish(z)
            time.sleep(0.02)


# ===========================================================================
# CLI — prove every primitive in isolation. You watch; hand on the e-stop.
# ===========================================================================
def _countdown(msg, secs=3):
    print(f"\n>>> {msg}")
    for i in range(secs, 0, -1):
        print(f"    starting in {i} ... (Ctrl-C to abort)", end="\r", flush=True)
        time.sleep(1)
    print(" " * 60, end="\r")


def _run_demo(drive):
    """Scripted sequence that exercises EVERY primitive, with pauses to watch."""
    seq = [
        ("FORWARD  0.15 m/s",  lambda: drive.drive_for( 0.15,  0.0, 3.0)),
        ("BACK     0.15 m/s",  lambda: drive.drive_for(-0.15,  0.0, 3.0)),
        ("SPIN LEFT  0.4 rad/s (in place)",  lambda: drive.drive_for(0.0,  0.4, 3.0)),
        ("SPIN RIGHT 0.4 rad/s (in place)",  lambda: drive.drive_for(0.0, -0.4, 3.0)),
        ("ARC forward-left (0.15, +0.3)",    lambda: drive.drive_for(0.15, 0.3, 3.0)),
        ("ARC forward-right (0.15, -0.3)",   lambda: drive.drive_for(0.15,-0.3, 3.0)),
    ]
    print("\n=== L0 LOCOMOTION DEMO — proving each primitive ===")
    print("Watch the robot. Confirm each motion matches the label.\n")
    for label, action in seq:
        _countdown(label, 3)
        print(f"    >>> {label}")
        action()
        print("    stopped. (1.5 s pause)")
        time.sleep(1.5)
    print("\n=== demo complete — every primitive exercised ===")


def main():
    import argparse
    p = argparse.ArgumentParser(
        description="robot_drive — isolated L0 locomotion test for the Scout Mini")
    p.add_argument("--topic", default="/cmd_vel")
    p.add_argument("--max-v", type=float, default=0.30, help="speed cap m/s")
    p.add_argument("--max-w", type=float, default=0.80, help="turn cap rad/s")
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_motion(name, help_):
        sp = sub.add_parser(name, help=help_)
        sp.add_argument("--v", type=float, default=0.15, help="linear m/s")
        sp.add_argument("--w", type=float, default=0.40, help="angular rad/s")
        sp.add_argument("--t", type=float, default=3.0, help="duration s")
        return sp

    add_motion("forward", "drive straight forward")
    add_motion("back",    "drive straight back")
    add_motion("left",    "spin in place, left (CCW)")
    add_motion("right",   "spin in place, right (CW)")
    add_motion("arc",     "curve: forward + turn together")
    sub.add_parser("stop", help="publish a stop")
    sub.add_parser("demo", help="run the full primitive sequence (watch it)")

    args = p.parse_args()

    rclpy.init()
    drive = RobotDrive(topic=args.topic, max_v=args.max_v, max_w=args.max_w)
    try:
        if args.cmd == "demo":
            _run_demo(drive)
        elif args.cmd == "stop":
            drive.hard_stop()
            print("stop published.")
        else:
            mapping = {
                "forward": ( abs(args.v),  0.0),
                "back":    (-abs(args.v),  0.0),
                "left":    ( 0.0,  abs(args.w)),
                "right":   ( 0.0, -abs(args.w)),
                "arc":     ( args.v, args.w),
            }
            v, w = mapping[args.cmd]
            _countdown(f"{args.cmd.upper()}  v={v:+.2f} w={w:+.2f}  for {args.t}s", 3)
            print(f"    >>> driving: v={v:+.2f} m/s  w={w:+.2f} rad/s  ({args.t}s)")
            drive.drive_for(v, w, args.t)
            print("    stopped.")
    except KeyboardInterrupt:
        print("\n!!! aborted — hard stop")
    finally:
        drive.hard_stop()
        drive.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
