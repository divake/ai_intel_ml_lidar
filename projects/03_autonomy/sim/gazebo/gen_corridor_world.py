#!/usr/bin/python3
"""Generate a Gazebo corridor world (rectangular LOOP with 4 corners) + the matching
teach path along the centerline. Loop layout mirrors the real scenario: a 1.8 m-wide
corridor around a core, with 90-degree corners (the part that breaks on the robot).

Outputs (in this dir):
  corridor.world      — SDF: physics + sensors + scene plugins, ground, light, walls
  teach_path_sim.csv  — centerline path (world frame; robot starts at (0,0) heading +x,
                        so KISS-ICP zeroed there => KISS frame == path frame)

Optional cavities (lab-door pockets) can be added later; start simple to validate first.
"""
import os

HERE = os.path.dirname(os.path.realpath(__file__))

# centerline loop corners (start at origin, go +x; CCW rectangle 12 x 8)
LOOP = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0), (0.0, 8.0), (0.0, 0.0)]
HALF = 0.9            # corridor half-width (=> 1.8 m wide)
TH = 0.10            # wall thickness
H = 1.5              # wall height
STEP = 0.30          # path waypoint spacing (m)


def wall(name, cx, cy, sx, sy):
    return f"""
    <model name="{name}"><static>true</static>
      <link name="l"><pose>{cx:.3f} {cy:.3f} {H/2:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {H:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {H:.3f}</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link></model>"""


def rect_walls(prefix, x0, y0, x1, y1):
    w = []
    w.append(wall(f"{prefix}_bot", (x0 + x1) / 2, y0, x1 - x0 + TH, TH))
    w.append(wall(f"{prefix}_top", (x0 + x1) / 2, y1, x1 - x0 + TH, TH))
    w.append(wall(f"{prefix}_lft", x0, (y0 + y1) / 2, TH, y1 - y0 + TH))
    w.append(wall(f"{prefix}_rgt", x1, (y0 + y1) / 2, TH, y1 - y0 + TH))
    return w


def gen_world():
    xs = [p[0] for p in LOOP]; ys = [p[1] for p in LOOP]
    walls = []
    walls += rect_walls("outer", min(xs) - HALF, min(ys) - HALF, max(xs) + HALF, max(ys) + HALF)
    walls += rect_walls("inner", min(xs) + HALF, min(ys) + HALF, max(xs) - HALF, max(ys) - HALF)
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="corridor">
    <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction></light>
    <model name="ground"><static>true</static>
      <link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link></model>
    {''.join(walls)}
  </world>
</sdf>
"""
    with open(os.path.join(HERE, "corridor.world"), "w") as f:
        f.write(sdf)
    print("wrote corridor.world  (", len(walls), "walls )")


def gen_path():
    import math
    pts = []
    for i in range(len(LOOP) - 1):
        x0, y0 = LOOP[i]; x1, y1 = LOOP[i + 1]
        d = math.hypot(x1 - x0, y1 - y0); n = max(1, int(d / STEP))
        for k in range(n):
            t = k / n
            pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    with open(os.path.join(HERE, "teach_path_sim.csv"), "w") as f:
        f.write("# x y yaw  (sim corridor centerline; robot starts at (0,0) heading +x)\n")
        for i, (x, y) in enumerate(pts):
            nx, ny = pts[(i + 1) % len(pts)]
            yaw = math.atan2(ny - y, nx - x)
            f.write(f"{x:.3f} {y:.3f} {yaw:.4f}\n")
    print("wrote teach_path_sim.csv  (", len(pts), "waypoints )")


if __name__ == "__main__":
    gen_world()
    gen_path()
