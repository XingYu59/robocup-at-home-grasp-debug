#!/usr/bin/env python3
"""
顶抓几何自检 —— 在**不启动 Gazebo / MoveIt** 的前提下，用 URDF 的真实碰撞几何
回答一个问题：给某个物体、某个抓取高度（grasp_lift）、某个采样角，手的碰撞体
会不会撞到物体 / 桌面？

为什么需要它（2026-09-14 定位到的真实故障）：
    日志里 MTC 的 ComputeIK 报 `grasp pose IK (0/25): eef in collision: fr3_hand - object`
    → 25 个采样角全被拒。查 MTC 源码（core/src/stages/compute_ik.cpp）：
    `isTargetPoseCollidingInEEF()` 会把**手部组的碰撞体按目标位姿摆好**，在
    "allow collision (hand,object)" 生效**之前**做一次碰撞检查 ✗
    → 抓取位姿本身如果让手和物体相交，25 个角全部无解，与视觉精度、真值、定位都无关。

结论（本脚本可复现）：老配置把 `grasp_lift` 当成"箱心上方 3 cm"，而手部本体
（fr3_hand 的碰撞网格）只伸到 **指尖平面上方 37.4 mm**，指腹（橡胶指尖）的
有效接触面只覆盖 **指尖平面上下 ±9 mm**。于是：
    物体顶面 − 指尖平面 > 37.4 mm  →  物体顶段插进手掌 → fr3_hand 撞物体 ✗
    物体顶面 − 指尖平面 <  9.0 mm  →  指尖悬在物体顶上，什么都没夹到 ✗
即 **grasp_lift 必须由"物体顶面"反推**，而不是"取高度的 1/4"。

用法（工作区已 build 过，需 source install/setup.bash 让 xacro 找到包）：
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 src/HANDOFF_harness/check_grasp_geometry.py            # 全表体检
    python3 src/HANDOFF_harness/check_grasp_geometry.py mustard_bottle
    python3 src/HANDOFF_harness/check_grasp_geometry.py --sweep coke can   # 打印可行高度窗
"""

import argparse
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

# ── 夹爪/手部的几何常量（由本脚本从 URDF 实测，见 verify_derived_constants()）──
FINGER_JOINT_Z = 0.0584          # fr3_finger_joint 在 hand 系里的 z（手指根部）
TCP_Z = 0.1034                   # fr3_hand_tcp 在 hand 系里的 z（指尖平面）
Q_OPEN = 0.04                    # SRDF group_state "open"：每根手指的位移
MAX_GAP = 2 * Q_OPEN             # 开口 0.08
GRIP_MARGIN = 0.005              # 开口校验留的余量（窄边 ≤ 0.075）

# 手部组的碰撞体（URDF 里 fr3_hand 是网格、两根手指是 4 个盒子）
HAND_LINKS = ("fr3_hand", "fr3_leftfinger", "fr3_rightfinger")


# ══════════════════════ 小工具：SE(3) ══════════════════════

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(r, p, y):
    return rot_z(y) @ rot_y(p) @ rot_x(r)          # URDF 约定：Rz·Ry·Rx


def T(R=np.eye(3), t=(0.0, 0.0, 0.0)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def trans(xyz):
    return T(t=(xyz))


def parse_xyz(s, default=(0.0, 0.0, 0.0)):
    if not s:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in s.split()], dtype=float)


def parse_rpy(s):
    if not s:
        return np.zeros(3)
    return np.array([float(v) for v in s.split()], dtype=float)


# ══════════════════════ 几何元素 ══════════════════════

class OBB:
    """有向盒：中心 + 三个正交轴（列向量）+ 半边长。"""

    def __init__(self, M, half):
        self.c = M[:3, 3].copy()
        self.R = M[:3, :3].copy()
        self.half = np.asarray(half, dtype=float)

    @property
    def corners(self):
        out = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    out.append(self.c + self.R @ (self.half * np.array([sx, sy, sz])))
        return np.array(out)


class Mesh:
    """三角网格（只存三角面片的顶点）。"""

    def __init__(self, M, tris):
        self.M = M
        self.tris = tris                       # (n,3,3)，link 系下

    def world_tris(self):
        return self.tris @ self.M[:3, :3].T + self.M[:3, 3]


def read_stl_binary(path):
    """二进制 STL → (n,3,3) 三角面片。

    ★ 每条 facet 记录 50 字节 = 12(法向) + 36(三个顶点) + 2(属性)，
      取顶点必须从偏移 12 开始；从 0 开始会把法向当成顶点（法向是单位向量，
      于是包围盒变成 ±1 ✗ 实测踩过）。
    """
    with open(path, "rb") as fh:
        fh.read(80)
        n = struct.unpack("<I", fh.read(4))[0]
        tris = np.empty((n, 3, 3), dtype=float)
        for i in range(n):
            d = fh.read(50)
            tris[i] = np.frombuffer(d[12:48], dtype="<f4").reshape(3, 3)
    return tris


def mesh_path_from_uri(uri, src_root):
    """package://pkg/... → 源码树里的真实路径。"""
    m = re.match(r"package://([^/]+)/(.*)", uri)
    if not m:
        return None
    pkg, rest = m.group(1), m.group(2)
    p = os.path.join(src_root, pkg, rest)
    return p if os.path.isfile(p) else None


# ══════════════════════ 碰撞判定 ══════════════════════

def obb_vs_aabb(obb, half):
    """OBB vs 以原点为中心、半边长 half 的 AABB（SAT，15 根轴）。"""
    R = obb.R
    abc = obb.half
    e = np.abs(R) @ abc + 1e-12
    # 3 根 AABB 轴
    if np.any(np.abs(obb.c) > half + e):
        return False
    # 3 根 OBB 轴
    t = np.abs(R.T @ obb.c)
    if np.any(t > abc + np.abs(R.T) @ half + 1e-12):
        return False
    # 9 根叉积轴
    for i in range(3):
        for j in range(3):
            ai = np.array([R[0, i], R[1, i], R[2, i]])
            bj = np.zeros(3)
            bj[j] = 1.0
            axis = np.cross(ai, bj)
            n = np.linalg.norm(axis)
            if n < 1e-9:
                continue
            axis = axis / n
            ra = sum(abc[k] * abs(np.dot(axis, R[:, k])) for k in range(3))
            rb = sum(half[k] * abs(axis[k]) for k in range(3))
            if abs(np.dot(axis, obb.c)) > ra + rb + 1e-12:
                return False
    return True


def tri_vs_aabb(tri, half, eps=1e-9):
    """三角形 vs AABB：Akenine-Möller 的 13 轴 SAT（精确，不采样）。"""
    # 把三角形平移到盒子中心为原点
    v = np.asarray(tri, dtype=float)
    mn = v.min(axis=0)
    mx = v.max(axis=0)
    if np.any(mn > half + eps) or np.any(mx < -half - eps):
        return False
    v = v - 0.0
    f = v - v[0]                                     # 边向量
    # 9 根轴：box 轴 × 三角形边
    for i in range(3):
        for j in range(3):
            a = np.zeros(3)
            a[i] = 1.0
            axis = np.cross(a, f[j])
            n = np.linalg.norm(axis)
            if n < 1e-12:
                continue
            axis = axis / n
            p = np.array([np.dot(axis, v[k]) for k in range(3)])
            r = half[0] * abs(axis[0]) + half[1] * abs(axis[1]) + half[2] * abs(axis[2])
            if p.min() > r + eps or p.max() < -r - eps:
                return False
    # 三角形法向
    normal = np.cross(f[1], f[2])
    n = np.linalg.norm(normal)
    if n > 1e-12:
        normal = normal / n
        d = np.dot(normal, v[0])
        r = half[0] * abs(normal[0]) + half[1] * abs(normal[1]) + half[2] * abs(normal[2])
        if d > r + eps or d < -r - eps:
            return False
    return True


# ══════════════════════ URDF → 手部几何 ══════════════════════

class HandModel:
    """fr3_hand / fr3_leftfinger / fr3_rightfinger 的碰撞几何 + 关节运动学。"""

    def __init__(self, urdf_path, src_root):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            self.joints[j.get("name")] = j
        self.by_parent = {}
        for j in root.findall("joint"):
            self.by_parent.setdefault(j.find("parent").get("link"), []).append(j)
        self.links = {l.get("name"): l for l in root.findall("link")}
        self.src_root = src_root
        self._segments = self._build_segments()

    # ── 手部组里每个 link 相对 fr3_hand 的链 ──────────────────────
    def _build_segments(self):
        """返回 {link: [ (kind, ...), ...] }，每段带"hand 系下的变换(给定 q)"。"""
        segs = {}
        for link in HAND_LINKS:
            chain = self._chain_to_hand(link)
            if chain is None:
                continue
            items = []
            for el in self.links[link].findall("collision"):
                org = el.find("origin")
                xyz = parse_xyz(org.get("xyz") if org is not None else None)
                rpy = parse_rpy(org.get("rpy") if org is not None else None)
                Mc = T(rpy_to_R(*rpy), xyz)
                geom = el.find("geometry")
                box = geom.find("box")
                mesh = geom.find("mesh")
                if box is not None:
                    size = parse_xyz(box.get("size"))
                    items.append(("box", Mc, size / 2.0))
                elif mesh is not None:
                    mp = mesh_path_from_uri(mesh.get("filename"), self.src_root)
                    if mp is None:
                        raise RuntimeError("找不到网格: " + str(mesh.get("filename")))
                    items.append(("mesh", Mc, read_stl_binary(mp)))
            segs[link] = (chain, items)
        return segs

    def _chain_to_hand(self, link):
        """从 fr3_hand 走到 link 的关节序列（含 prismatic 位移由 q 决定）。"""
        # 只处理 hand→finger 的一层（本工程就是一层）
        for j in self.by_parent.get("fr3_hand", []):
            if j.find("child").get("link") == link:
                return [j]
        if link == "fr3_hand":
            return []
        return None

    # ── 给定 q，把整只手摆到"object 系"里 ─────────────────────────
    def place(self, T_ob_hand, q):
        """返回 (boxes, tris)：box 用 OBB 表示，网格用世界(物体)系下的三角面片。"""
        boxes, tris = [], []
        for link, (chain, items) in self._segments.items():
            M = T_ob_hand.copy()
            for j in chain:
                org = j.find("origin")
                xyz = parse_xyz(org.get("xyz") if org is not None else None)
                rpy = parse_rpy(org.get("rpy") if org is not None else None)
                M = M @ T(rpy_to_R(*rpy), xyz)
                jt = j.get("type")
                if jt == "prismatic":
                    axis_el = j.find("axis")
                    axis = parse_xyz(axis_el.get("xyz") if axis_el is not None else None,
                                     (1.0, 0.0, 0.0))
                    M = M @ T(t=axis * q)
            for kind, Mc, data in items:
                Mw = M @ Mc
                if kind == "box":
                    boxes.append(OBB(Mw, data))
                else:
                    tris.append(data @ Mw[:3, :3].T + Mw[:3, 3])
        return boxes, tris


def T_object_from_hand(lift, theta):
    """抓取位姿：指尖平面在箱心上方 lift 处，绕物体 z 采样 theta。

    推导（与 pick_and_place.cpp / MTC ComputeIK 的语义一致）：
      ik_frame = fr3_hand_tcp 沿自身 +z 偏 lift，且必须落在物体箱心；
      target_pose = Rz(theta)（GenerateGraspPose 只转不移动）
      ⇒ R_hand = Rz(theta)·Rx(pi)      （手 +z 朝下 = 顶抓，指尖轴水平）
      ⇒ p_tcp  = (0, 0, lift)          （物体系）
    """
    R = rot_z(theta) @ rot_x(math.pi)
    p_tcp = np.array([0.0, 0.0, lift])
    p_hand = p_tcp - R @ np.array([0.0, 0.0, TCP_Z])
    return T(R, p_hand)


# ══════════════════════ 体检 ══════════════════════

def check_pose(hand, lift, theta, half, table_check=True):
    """返回 (与物体相撞的部件列表, 是否撞桌面)。"""
    T_ob_hand = T_object_from_hand(lift, theta)
    boxes, tris = hand.place(T_ob_hand, Q_OPEN)
    hits = []
    for b in boxes:
        if obb_vs_aabb(b, half):
            hits.append("finger_box")
    for mesh in tris:
        for tri in mesh.reshape(-1, 3, 3):
            if tri_vs_aabb(tri, half):
                hits.append("fr3_hand(mesh)")
                break
    table = False
    if table_check:
        zmin = min([b.c[2] - np.sum(np.abs(b.R[2, :]) * b.half) for b in boxes] +
                   [m.reshape(-1, 3)[:, 2].min() for m in tris])
        table = zmin < -half[2]
    return hits, table


def closing_span(depth, width, theta):
    """两根手指闭合方向上的物体投影宽度（= 需要的开口）。"""
    return depth * abs(math.sin(theta)) + width * abs(math.cos(theta))


def feasible_lifts(hand, obj, step=0.002, angles=None, table_check=True):
    """扫高度：返回 {lift: (可行角数, 撞物体的角数, 撞桌面的角数)}。"""
    depth, width, height = obj["depth"], obj["width"], obj["height"]
    half = np.array([depth / 2.0, width / 2.0, height / 2.0])
    angles = angles if angles is not None else [k * math.pi / 12.0 for k in range(24)]
    out = {}
    lift = -0.02
    while lift <= height / 2.0 + 0.06:
        good = bad = tab = 0
        for th in angles:
            hits, t = check_pose(hand, lift, th, half, table_check)
            if hits:
                bad += 1
            elif t:
                tab += 1
            else:
                good += 1
        out[round(lift, 4)] = (good, bad, tab)
        lift += step
    return out


def load_objects(path):
    import yaml
    doc = yaml.safe_load(open(path, encoding="utf-8"))
    return doc["objects"]


# 橡胶指尖那块碰撞盒的半边长（URDF: 17.5e-3 x 15.2e-3 x 18.5e-3）——用它认出来
TIP_HALF = (0.00875, 0.0076, 0.00925)


def derive_constants(hand):
    """量三件事：槽内净空 / 槽外掌面 / 指腹接触窗（都由 URDF 真实几何算）。

    ★ 关键区别（2026-09-14 量错过的就是它）：
        |y| < 40 mm = 两指之间的【槽】→ 物体从这里插进去，净空 +102.2 mm
        |y| > 40 mm = 手指安装座那一带 → 只有"胖物体"才会碰到，净空 +37.4 mm
      所有可夹类别窄边 ≤ 75 mm（半宽 ≤ 37.5 mm）→ 整个物体都在槽里，
      所以约束是【槽内净空】，不是掌面 ✗
    """
    boxes, tris = hand.place(T_object_from_hand(0.0, 0.0), Q_OPEN)
    pts = np.concatenate([m.reshape(-1, 3) for m in tris])              # 手部网格（物体系）
    slot = pts[np.abs(pts[:, 1]) < 0.040]                              # 两指之间那条带
    outer = pts[np.abs(pts[:, 1]) >= 0.040]
    tip_lo, tip_hi = 1e9, -1e9
    for b in boxes:
        if np.allclose(b.half, TIP_HALF, atol=1e-9):
            zs = b.corners[:, 2]
            tip_lo, tip_hi = min(tip_lo, zs.min()), max(tip_hi, zs.max())
    return {"slot_obj_z": float(slot[:, 2].min()) if len(slot) else TCP_Z,
            "palm_outer_obj_z": float(outer[:, 2].min()) if len(outer) else TCP_Z,
            "tip_top_obj_z": tip_hi,               # 物体顶面应高于它（否则指尖悬空）
            "tip_bottom_obj_z": tip_lo}


def grip_window(obj, consts, table_margin=0.005, slot_margin=0.010):
    """由几何反推出的可行 grasp_lift 窗口（箱心为 0，向上为正）。

    三个约束（物体系；物体顶面 = +h/2；指尖平面在箱心上方 lift）：
      ① 不顶到槽顶：物体顶面 ≤ 指尖平面 + 槽内净空(102.2 mm) − 余量  → lift ≥ h/2 − (slot−margin)
      ② 夹得住：物体顶面 ≥ 指尖平面 + 指腹上沿(9.0 mm)              → lift ≤ h/2 − tip_top
      ③ 不撞桌面：指尖最低点离桌面 ≥ table_margin                   → lift ≥ margin − h/2 − tip_bottom
    """
    h = obj["height"]
    lo = max(h / 2.0 - (consts["slot_obj_z"] - slot_margin),
             table_margin - h / 2.0 - consts["tip_bottom_obj_z"])
    hi = h / 2.0 - consts["tip_top_obj_z"]
    return lo, hi


def recommend(obj, consts, sweep, need=8):
    """在"指腹能贴到物体侧面"的前提下，取【可行角 ≥ need 的最低高度】。

    为什么是"最低"：夹持点越低越靠近重心 → 抗倾覆力臂越大、越不容易被推倒 ✓
    为什么不能更低：手部碰撞网格的净空随物体脚印宽度变化（见文件头），
    低于某个高度就会撞到掌心/肩部三角面 ✗ —— 这个下沿只能用精确碰撞扫描量出来，
    不能靠"夹重心"想当然（实测过：想当然取 0 → 24 个角全撞）。
    """
    lo, hi = grip_window(obj, consts)
    if hi < lo:
        return None, 0, (lo, hi)
    for n in (need, 1):
        for l in sorted(sweep):
            if l > hi + 1e-9:
                break
            if sweep[l][0] >= n:
                return l, sweep[l][0], (lo, hi)
    return None, 0, (lo, hi)


def main():
    ap = argparse.ArgumentParser(description="顶抓几何自检")
    ap.add_argument("classes", nargs="*", help="只查这些类别（默认全表）")
    ap.add_argument("--sweep", action="store_true", help="打印每个高度上的可行角数")
    ap.add_argument("--src-root", default=None)
    args = ap.parse_args()

    src_root = args.src_root or os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urdf = "/tmp/dsh_check_robot.urdf"
    if not os.path.isfile(urdf):
        import xacro
        doc = xacro.process_file(
            os.path.join(src_root, "turtlebot3_manipulation_gazebo", "urdf",
                         "turtlebot3_manipulation.urdf.xacro"),
            mappings={"use_sim": "true"})
        open(urdf, "w").write(doc.toxml())
    hand = HandModel(urdf, src_root)
    c = derive_constants(hand)
    print("=" * 96)
    print("手部几何实测（fr3_hand，握爪张开 2×{:.0f} mm）：".format(Q_OPEN * 1000))
    # ★ 2026-09-19 修：这两个常数本来就量在【物体系、且手摆在 lift=0（指尖平面=箱心）】，
    #   所以它们**本身就是"指尖平面上方多少毫米"**，不能再减 TCP_Z ✗
    #   （旧打印减了 TCP_Z ⇒ 打出 6.2 / 66.0 mm 两个假数：前者应为 97.2（中央槽净空），
    #     后者应为 37.4（手指安装座那一带）—— objects.yaml 的注释里引用的正是 37.4/97 ✓）
    print("  槽内净空（|y| < 40 mm，两指之间、物体插进来的地方）: 指尖平面上方 {:.1f} mm".format(
        c["slot_obj_z"] * 1000))
    print("  槽外掌面（|y| > 40 mm，手指安装座那一带）:           指尖平面上方 {:.1f} mm".format(
        c["palm_outer_obj_z"] * 1000))
    print("  指腹（橡胶指尖）接触窗：指尖平面上方 {:.1f} mm ~ 下方 {:.1f} mm".format(
        c["tip_top_obj_z"] * 1000, -c["tip_bottom_obj_z"] * 1000))
    print("  ⇒ 【物体顶面】最高只能到指尖平面上方 {:.1f} mm（= 手指安装座那一带的净空）".format(
        c["palm_outer_obj_z"] * 1000))
    print("     —— 这就是「抓取高度」的几何上限：物体越高，指尖平面就越下不去 ✓")
    print("=" * 96)

    objs = load_objects(os.path.join(src_root, "turtlebot3_manipulation_grasp",
                                     "config", "objects.yaml"))
    names = args.classes or list(objs.keys())
    print("{:>18} {:>5} {:>5} {:>5} | {:>6} {:>4} | {:>6} {:>6} | {:6} {:>4} | {:>8} | {}".format(
        "class", "d", "w", "h", "lift", "角", "窗下界", "窗上界", "建议", "角",
        "下沿余量", "结论"))
    for name in names:
        if name not in objs:
            print("  ! 目录表里没有 {}".format(name))
            continue
        o = objs[name]
        step = 0.001 if args.sweep else 0.002
        sweep = feasible_lifts(hand, o, step=step)
        cur = o.get("grasp_lift", 0.0)
        near = min(sweep, key=lambda k: abs(k - cur))
        good_now = sweep[near][0]
        rec, rec_ok, (lo, hi) = recommend(o, c, sweep)
        # ★ 2026-09-19 新增：【下沿余量】= 当前 lift 离"还能用的最低高度"有多少毫米。
        #   为什么必须看它：手部碰撞网格（fr3_hand）在指尖平面上方只有 37.4 mm
        #   ⇒ 物体越高，"手不撞物体"的下沿就越靠近当前值；贴着下沿取值时，
        #   臂执行 ±2 mm / 视觉 z 误差几 mm 就会让手部本体蹭到物体顶面把它推走 ✗
        #   （potted_meat_can 的 0.004 就只剩 0.4 mm ⇒ 现场"手停在罐顶、合爪空合"）
        #   余量为负 = 当前值本身不可行（24 个角全撞）✗
        lower = None
        for l in sorted(sweep):
            if sweep[l][0] >= 8:
                lower = l
                break
        marg = None if lower is None else cur - lower
        marg_s = "-" if marg is None else "{:+.1f}mm".format(marg * 1000.0)
        if not o.get("graspable", True):
            verdict = "— 不可夹（表里 graspable=false，该值不参与抓取）"
        elif good_now >= 8:
            verdict = "✓ 可用（{} 个采样角可行）".format(good_now)
            if marg is not None and marg < 0.005:
                verdict += " ⚠ 但贴着下沿（余量 <5 mm）：手部本体离物体顶面太近 ⇒ 建议抬到 {:.3f} ✓".format(
                    min(lower + 0.005, hi) if lower is not None and hi > lower else cur)
        elif good_now >= 1:
            verdict = "⚠ 只有 {} 个采样角可行（能规划，但对停位/摆放角度敏感）".format(good_now)
        elif rec is None:
            verdict = "✗ 窗口内没有可行高度（物体太高/太扁，考虑换物体或侧抓）"
        else:
            verdict = "✗ 当前值 {:.0f} 个角可行 → 应改成 {:.3f}（{} 个角可行）".format(
                good_now, rec, rec_ok)
        print("{:>18} {:5.3f} {:5.3f} {:5.3f} | {:6.3f} {:4d} | {:6.3f} {:6.3f} | {:6} {:4} "
              "| {:>8} | {}".format(
                  name, o["depth"], o["width"], o["height"], cur, good_now, lo, hi,
                  "-" if rec is None else "{:.3f}".format(rec), rec_ok, marg_s, verdict))
        if args.sweep:
            for l in sorted(sweep):
                g, b, t = sweep[l]
                mark = ""
                if abs(l - cur) < 5e-4:
                    mark = "  ← 当前值"
                if rec is not None and abs(l - rec) < 5e-4:
                    mark += "  ← 建议"
                print("      lift={:+.3f}  可行角 {:2d}  撞物体 {:2d}  撞桌面 {:2d}{}".format(
                    l, g, b, t, mark))
    print("=" * 96)
    print("说明：")
    print("  * 角 = 24 个采样角（15° 一步）里「手不撞物体也不撞桌面」的个数；")
    print("    MTC 会自己扫这些角，只要有几个可行就能解出 IK（≥8 个算稳）。")
    print("  * 撞物体的主力是【方盒碰撞体在斜角上的角】：方形足迹的物体（罐/苹果/碗）")
    print("    只有在闭合轴对齐盒面时才夹得进去 → 这是「开口 0.08 夹方盒」的固有现象。")
    print("  * 开口校验（min(depth,width) ≤ {:.3f}）由 grasp_node 判定，本脚本只判碰撞。".format(
        MAX_GAP - GRIP_MARGIN))


if __name__ == "__main__":
    sys.exit(main())
