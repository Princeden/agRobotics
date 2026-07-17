#!/usr/bin/env python3
"""Find a UR5e start pose whose wrist camera looks at the ArUco marker AND stays
well clear of singularities and joint limits (so Servo can actually move through
a useful range instead of stalling).

FK is computed from the real expanded URDF chain base_link ->
camera_color_optical_frame, so there are no DH/frame-convention assumptions.

Usage:
  python3 find_start_pose.py            # search + print
  python3 find_start_pose.py --write    # also overwrite config/servo_start_positions.yaml
"""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(
    os.path.dirname(PKG),
    "ur5e_visual_servo_sim/ur_simulation_gazebo/urdf/ur_with_camera.urdf.xacro",
)
MARKER = np.array([0.6, 0.0, 0.4])   # world position of the marker
TARGET_DIST = 0.5                    # match pbvs standoff
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def expand_urdf():
    out = subprocess.run(
        ["/opt/ros/humble/bin/xacro", XACRO, "name:=ur", "ur_type:=ur5e",
         "camera_sensor_type:=camera"],
        check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_R(axis, q):
    ax = np.array(axis) / np.linalg.norm(axis)
    x, y, z = ax
    c, s, C = np.cos(q), np.sin(q), 1 - np.cos(q)
    return np.array([[c + x*x*C, x*y*C - z*s, x*z*C + y*s],
                     [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
                     [z*x*C - y*s, z*y*C + x*s, c + z*z*C]])


def H(R, p):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M


root = expand_urdf()
joints, children, limits = {}, {}, {}
for j in root.findall("joint"):
    name, jtype = j.get("name"), j.get("type")
    parent, child = j.find("parent").get("link"), j.find("child").get("link")
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
    ax = j.find("axis")
    axis = [float(v) for v in (ax.get("xyz", "1 0 0").split() if ax is not None else [1, 0, 0])]
    joints[name] = dict(type=jtype, parent=parent, child=child, xyz=xyz, rpy=rpy, axis=axis)
    children.setdefault(parent, []).append(name)
    lim = j.find("limit")
    if lim is not None:
        limits[name] = (float(lim.get("lower", -6.28)), float(lim.get("upper", 6.28)))


def path_to(target):
    from collections import deque
    q, seen = deque([("base_link", [])]), {"base_link"}
    while q:
        link, jp = q.popleft()
        if link == target:
            return jp
        for jn in children.get(link, []):
            c = joints[jn]["child"]
            if c not in seen:
                seen.add(c); q.append((c, jp + [jn]))
    return None


CHAIN_CAM = path_to("camera_color_optical_frame")
CHAIN_TOOL = path_to("tool0")
assert CHAIN_CAM and CHAIN_TOOL


def fk(chain, qd):
    M, info = np.eye(4), []
    for jn in chain:
        j = joints[jn]
        M = M @ H(rpy_to_R(*j["rpy"]), np.array(j["xyz"]))
        if j["type"] in ("revolute", "continuous"):
            if jn in ARM_JOINTS:
                z = M[:3, :3] @ (np.array(j["axis"]) / np.linalg.norm(j["axis"]))
                info.append((z, M[:3, 3].copy()))
            M = M @ H(axis_R(j["axis"], qd.get(jn, 0.0)), np.zeros(3))
    return M, info


def cond(qd):
    M, info = fk(CHAIN_TOOL, qd)
    pt = M[:3, 3]
    J = np.zeros((6, 6))
    for i, (z, p) in enumerate(info):
        J[:3, i] = np.cross(z, pt - p); J[3:, i] = z
    s = np.linalg.svd(J, compute_uv=False)
    return s[0] / s[-1] if s[-1] > 1e-9 else 1e9


def evaluate(q):
    qd = dict(zip(ARM_JOINTS, q))
    M, _ = fk(CHAIN_CAM, qd)
    cam_p, opt_z = M[:3, 3], M[:3, 2]
    d = MARKER - cam_p
    dist = np.linalg.norm(d)
    aim = np.degrees(np.arccos(np.clip(opt_z @ (d / dist), -1, 1)))
    k = cond(qd)
    # margin to nearest joint limit (rad)
    margin = min(min(q[i] - limits[ARM_JOINTS[i]][0], limits[ARM_JOINTS[i]][1] - q[i])
                 for i in range(6))
    c = (3.0 * aim + 60 * abs(dist - TARGET_DIST) + 0.6 * max(0, k - 20)
         + 80 * max(0, 0.5 - margin)            # stay >=0.5 rad from any limit
         + 30 * (abs(q[2]) < 0.4) + 30 * (abs(q[4]) < 0.4))  # elbow/wrist_2 off zero
    return c, dict(aim=aim, dist=dist, cond=k, margin=margin, cam_p=cam_p)


rng = np.random.default_rng(1)
seed = np.array([0.0, -1.0, 1.2, -1.4, 1.4, 0.0])
best = None
for k in range(40000):
    q = seed.copy() if k == 0 else seed + rng.uniform(-1.4, 1.4, 6)
    if k:
        q[0] = rng.uniform(-0.5, 0.5)
    c, info = evaluate(q)
    if best is None or c < best[0]:
        best = (c, q.copy(), info)

c, q, info = best
print("=== best start pose ===")
print(f"aim error   : {info['aim']:.1f} deg  (camera axis vs. line to marker; FOV half ~34)")
print(f"distance    : {info['dist']:.3f} m   (target {TARGET_DIST})")
print(f"cond number : {info['cond']:.1f}      (servo decel ~>100, halt ~>200)")
print(f"limit margin: {info['margin']:.2f} rad (nearest joint limit)")
print(f"camera pos  : [{info['cam_p'][0]:.3f} {info['cam_p'][1]:.3f} {info['cam_p'][2]:.3f}]")
print()
for n, v in zip(ARM_JOINTS, q):
    print(f"  {n:<20}{v: .4f}")

if "--write" in sys.argv:
    path = os.path.join(PKG, "config", "servo_start_positions.yaml")
    with open(path, "w") as f:
        f.write("# Auto-generated by scripts/find_start_pose.py\n")
        f.write("# Camera aims at the marker, well clear of singularities & joint limits.\n")
        f.write(f"# aim={info['aim']:.1f}deg dist={info['dist']:.2f}m cond={info['cond']:.0f} "
                f"limit_margin={info['margin']:.2f}rad\n")
        for n, v in zip(ARM_JOINTS, q):
            f.write(f"{n}: {v:.4f}\n")
    print(f"\nwrote {path}")
