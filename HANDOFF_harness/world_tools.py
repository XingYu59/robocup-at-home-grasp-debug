#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""换世界（比赛当天裁判给新 world）的两件工具，**不用起仿真**：

    python3 HANDOFF_harness/world_tools.py check <world文件>      # 体检 + 打印"该改哪一行、改成什么"
    python3 HANDOFF_harness/world_tools.py map   <world文件> [输出目录]
                                                                  # 直接从 world 生成 map.pgm/yaml
                                                                  # （不用开车跑 SLAM ✓ 秒级 ✓）

为什么要有它们（比赛现场时间最贵）：
    换一个 world 需要同步的东西散在 5 个地方：launch（world 路径+出生位姿）、
    grasp_params.yaml（抓哪张桌子 support_surface）、grasp_phase.py（AMCL 初始位姿
    INITIAL_X/Y/YAW、邻居桌 NEIGHBOR_TABLES）、nav2 的 map.pgm/yaml、objects.yaml（物体目录）。
    `check` 一条命令把这些**逐条核对并给出要粘贴的值** ✓；
    `map` 直接把 world 里的静态碰撞体栅格化成地图 ⇒ 定位/导航立刻可用 ✓
    （比"开车跑一遍 SLAM 再存图"快太多，而且和 world 完全一致 ✓）

解析范围（够用即止）：
    * `<include><uri>model://…</uri>` 与内联 `<model>`；`<pose>x y z r p y</pose>`
    * 模型 SDF 里的 `<collision>`：**盒体**（`<box><size>`）与网格（取不到就跳过并在报告里点名 ✗）
    * 表格类判定：有一块"薄而大"的盒体（z 厚 ≤0.12 m、x/y ≥0.4 m、且 0.5 m < z < 1.0 m）✓
    * 物体类判定：模型名（或 uri 末段）能对上 objects.yaml 的类别名 ✓
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, ".."))


# ══════════════════ 基本工具 ══════════════════
def _find_model_dirs():
    """Gazebo 找 model:// 的目录集合（与 launch 里的 IGN_GAZEBO_RESOURCE_PATH 对齐 ✓）。"""
    dirs = [os.path.join(SRC, "wpr_simulation_ros2", "models"),
            os.path.join(SRC, "turtlebot3_manipulation_gazebo", "models"),
            os.path.join(SRC, "wpr_simulation_ros2", "models", "wpr_simulation_assets")]
    for d in (os.environ.get("IGN_GAZEBO_RESOURCE_PATH") or "").split(os.pathsep):
        if d and d not in dirs:
            dirs.append(d)
    return [d for d in dirs if os.path.isdir(d)]


def resolve_model(uri):
    """model://A/B 或 model://name → SDF 文件路径（找不到返回 None）。"""
    m = re.match(r"model://([^/]+)(?:/(.*))?$", uri or "")
    if not m:
        return None
    first, rest = m.group(1), (m.group(2) or "")
    for d in _find_model_dirs():
        cands = []
        if rest:
            cands.append(os.path.join(d, first, rest))
        cands.append(os.path.join(d, first, "model.sdf"))
        cands.append(os.path.join(d, first, "model.config"))
        for c in cands:
            if os.path.isfile(c):
                if c.endswith(".config"):
                    try:
                        root = ET.parse(c).getroot()
                        sdf = root.find("sdf")
                        if sdf is not None and sdf.get("version"):
                            p = os.path.join(os.path.dirname(c), sdf.get("version"))
                            if os.path.isfile(p):
                                return p
                    except Exception:
                        pass
                    p = os.path.join(os.path.dirname(c), "model.sdf")
                    return p if os.path.isfile(p) else None
                return c
    return None


def parse_pose(s, default=(0.0,) * 6):
    if not s:
        return list(default)
    v = [float(x) for x in s.split()]
    return (v + list(default))[:6]


def box_corners_xy(cx, cy, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for dx in (-sx / 2.0, sx / 2.0):
        for dy in (-sy / 2.0, sy / 2.0):
            out.append((cx + c * dx - s * dy, cy + s * dx + c * dy))
    return out


def model_collisions(sdf_path):
    """模型 SDF 的全部盒体碰撞 → [(相对位姿 pose6, (sx,sy,sz))]；网格碰撞单独回报。"""
    boxes, meshes = [], []
    try:
        root = ET.parse(sdf_path).getroot()
    except Exception:
        return boxes, meshes
    for model in root.iter("model"):
        mpose = parse_pose((model.find("pose").text if model.find("pose") is not None else None))
        for link in model.iter("link"):
            lpose = parse_pose((link.find("pose").text if link.find("pose") is not None else None))
            for col in link.iter("collision"):
                cpose = parse_pose((col.find("pose").text if col.find("pose") is not None else None))
                g = col.find("geometry")
                if g is None:
                    continue
                b = g.find("box")
                me = g.find("mesh")
                rel = [mpose[i] + lpose[i] + cpose[i] for i in range(3)] + \
                      [mpose[3] + lpose[3] + cpose[3], 0.0, 0.0]
                if b is not None and b.find("size") is not None:
                    sz = [float(x) for x in b.find("size").text.split()]
                    boxes.append((rel, sz))
                elif me is not None:
                    meshes.append((rel, me.get("filename", "?")))
    return boxes, meshes


# ══════════════════ 读 world ══════════════════
class Entry:
    def __init__(self, name, pose, sdf_path, uri, boxes, meshes, static):
        self.name, self.pose, self.sdf_path = name, pose, sdf_path
        self.uri, self.boxes, self.meshes, self.static = uri, boxes, meshes, static

    @property
    def top_slab(self):
        """"薄而大、在桌面高度"的那块盒体 → (中心世界 xy, yaw, (sx,sy), z_top)；没有返回 None。"""
        best = None
        for (rel, sz) in self.boxes:
            sx, sy, szz = sz
            if szz <= 0.12 and sx >= 0.4 and sy >= 0.4 and 0.4 < rel[2] < 1.1:
                if best is None or sx * sy > best[1][0] * best[1][1]:
                    best = (rel, (sx, sy))
        if best is None:
            return None
        rel, (sx, sy) = best
        c, s = math.cos(self.pose[5]), math.sin(self.pose[5])
        wx = self.pose[0] + c * rel[0] - s * rel[1]
        wy = self.pose[1] + s * rel[0] + c * rel[1]
        return (wx, wy, self.pose[5], (sx, sy), self.pose[2] + rel[2] + szz_half(self, rel, sz=0.03))


def szz_half(*a, **k):        # 占位（top_slab 里只用来加半个厚度，下面重算）
    return 0.0


def read_world(path):
    """→ (entries, 世界边界 bbox, 缺模型的 uri 列表)"""
    root = ET.parse(path).getroot()
    world = root if root.tag == "world" else root.find("world")     # <sdf><world>… ✓
    if world is None:
        world = root
    entries, missing = [], []
    xs, ys = [], []
    for el in list(world):
        if el.tag == "include":
            uri = (el.find("uri").text or "").strip() if el.find("uri") is not None else ""
            name = (el.find("name").text or "").strip() if el.find("name") is not None else uri
            pose = parse_pose((el.find("pose").text if el.find("pose") is not None else None))
            sdf = resolve_model(uri)
            if sdf is None:
                missing.append(uri)
                continue
            boxes, meshes = model_collisions(sdf)
            static = True                     # include 的模型按静态处理（地图用）
            entries.append(Entry(name, pose, sdf, uri, boxes, meshes, static))
            xs.append(pose[0]); ys.append(pose[1])
        elif el.tag == "model":
            name = el.get("name") or "?"
            pose = parse_pose((el.find("pose").text if el.find("pose") is not None else None))
            static = (el.find("static") is not None)
            boxes, meshes = model_collisions_path_only(el)
            entries.append(Entry(name, pose, path, "", boxes, meshes, static))
            xs.append(pose[0]); ys.append(pose[1])
    bbox = (min(xs), min(ys), max(xs), max(ys)) if xs else (0, 0, 0, 0)
    return entries, bbox, missing


def model_collisions_path_only(model_el):
    """**内联在世界里的** <model>：它的 <pose> 就是世界位姿 ⇒ 这里不能再加进去 ✗
    （否则 top_slab 会把它再加一遍，坐标直接翻倍偏 —— 实测第一版就是这样 ✗）"""
    boxes, meshes = [], []
    mpose = [0.0] * 6                      # 世界位姿由 Entry.pose 负责 ✓
    for link in model_el.iter("link"):
        lpose = parse_pose((link.find("pose").text if link.find("pose") is not None else None))
        for col in link.iter("collision"):
            cpose = parse_pose((col.find("pose").text if col.find("pose") is not None else None))
            g = col.find("geometry")
            if g is None:
                continue
            rel = [mpose[i] + lpose[i] + cpose[i] for i in range(3)] + [mpose[3] + lpose[3] + cpose[3], 0, 0]
            if g.find("box") is not None and g.find("box").find("size") is not None:
                boxes.append((rel, [float(x) for x in g.find("box").find("size").text.split()]))
            elif g.find("mesh") is not None:
                meshes.append((rel, g.find("mesh").get("filename", "?")))
    return boxes, meshes


def load_object_keys():
    """objects.yaml 的类别名（归一化：小写、空格/下划线等价）→ 原始名"""
    p = os.path.join(SRC, "turtlebot3_manipulation_grasp", "config", "objects.yaml")
    out = {}
    try:
        import yaml
        doc = yaml.safe_load(open(p, encoding="utf-8")) or {}
        for k in (doc.get("objects") or {}):
            out[re.sub(r"[\s_]+", "", k.lower())] = k
    except Exception:
        pass
    return out


def norm(s):
    return re.sub(r"[\s_]+", "", (s or "").lower())


# ══════════════════ 子命令 ①：check ══════════════════
def cmd_check(world_path):
    keys = load_object_keys()
    entries, bbox, missing = read_world(world_path)
    print("=" * 92)
    print("世界文件: {}".format(os.path.abspath(world_path)))
    print("模型 {} 个；静态范围 x∈[{:.2f},{:.2f}] y∈[{:.2f},{:.2f}]".format(
        len(entries), bbox[0], bbox[2], bbox[1], bbox[3]))
    if missing:
        print("\n✗ 有 {} 个 model:// 解析不到（**Gazebo 启动会报错/模型消失**）：".format(len(missing)))
        for u in sorted(set(missing))[:20]:
            print("    {}".format(u))
        print("  → 把对应模型目录放进 wpr_simulation_ros2/models/ 或加进 IGN_GAZEBO_RESOURCE_PATH ✓")
    else:
        print("✓ 所有 model:// 都能在资源路径里解析到（不会缺模型 ✓）")

    tables, objs, others, meshy = [], [], [], []
    for e in entries:
        slab = e.top_slab
        cls = keys.get(norm(e.name)) or keys.get(norm(os.path.basename(e.uri.rstrip("/"))))
        if cls:
            objs.append((cls, e, slab))
        elif slab is not None:
            tables.append((e, slab))
        else:
            others.append(e)
        if e.meshes and not e.boxes:
            meshy.append(e.name)

    print("\n── 桌面（薄而大的盒体，0.4~1.1 m 高）────────────────────────────")
    if not tables:
        print("  （没找到桌面形状的模型 ⇒ 请手工确认哪张桌子放目标物体 ✗）")

    def _cnt(t):
        """这张桌子上有几个"能对上 objects.yaml 的物体" ✓"""
        e, (wx, wy, yaw, (sx, sy), _zt) = t
        n = 0
        for (cls, oe, _sl) in objs:
            dx, dy = oe.pose[0] - wx, oe.pose[1] - wy
            c, s2 = math.cos(-yaw), math.sin(-yaw)
            if abs(c * dx - s2 * dy) <= sx / 2 + 0.05 and abs(s2 * dx + c * dy) <= sy / 2 + 0.05:
                n += 1
        return n

    tables = sorted(tables, key=lambda t: -_cnt(t))
    for k, (e, slab) in enumerate(tables):
        wx, wy, yaw, (sx, sy), _ztop = slab
        n = _cnt((e, slab))
        if k == 0 and n > 0:
            print("  {:<22} 世界位姿 ({:+.3f},{:+.3f}, yaw={:+.3f})  桌面 {:.2f}×{:.2f} m"
                  "   ← ★ 物体最多（{} 个）⇒ 多半就是这次要抓的桌子".format(
                      e.name, e.pose[0], e.pose[1], e.pose[5], sx, sy, n))
            print("       → grasp_params.yaml 改成：")
            print("         support_surface.pose: [{:g}, {:g}, 0.765, 0.0, 0.0, {:g}]".format(wx, wy, yaw))
            print("         support_surface.length: {:g}   # x".format(sx))
            print("         support_surface.width:  {:g}   # y".format(sy))
        else:
            print("  {:<22} 世界位姿 ({:+.3f},{:+.3f}, yaw={:+.3f})  桌面 {:.2f}×{:.2f} m（物体 {} 个）"
                  .format(e.name, e.pose[0], e.pose[1], e.pose[5], sx, sy, n))

    print("\n── 桌上的物体（名字能对上 objects.yaml 的）────────────────────")
    if not objs:
        print("  （一个都没对上 ⇒ 要么这个 world 里没放物体，要么 objects.yaml 缺条目 ✗）")
    for (cls, e, slab) in objs:
        near = "（不在这张桌子上？）"
        for (t, ts) in tables:
            wx, wy, yaw, (sx, sy), _ = ts
            dx, dy = e.pose[0] - wx, e.pose[1] - wy
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx, ly = c * dx - s * dy, s * dx + c * dy
            if abs(lx) <= sx / 2 + 0.05 and abs(ly) <= sy / 2 + 0.05:
                near = "在 `{}` 上".format(t.name)
                break
        print("  {:<20} ← 模型 {:<22} 世界 ({:+.2f},{:+.2f}) {}".format(
            cls, e.name, e.pose[0], e.pose[1], near))

    print("\n── 疑似物体、但 objects.yaml 里没有（比赛 world 里很可能出现 ✗）────")
    unknown = []
    for e in others:
        if not e.boxes:
            continue
        big = max((sz[0] * sz[1] * sz[2], sz) for (_r, sz) in e.boxes)[1]
        if max(big) > 0.35:                    # 大块头 = 家具，不是物体 ✓
            continue
        unknown.append((e, big))
    if not unknown:
        print("  （没有 ✓ —— 这个 world 的物体都在目录表里）")
    for (e, sz) in unknown[:12]:
        # 目录表约定：depth=x, width=y, height=z（物体按出厂姿态摆 ✓）
        print('  "{}":'.format(e.name))
        print("    height: {:.4f}    depth: {:.4f}    width: {:.4f}".format(sz[2], sz[0], sz[1]))
        print("    grasp_lift: 0.000   # ⚠ 用 check_grasp_geometry.py <类别> 扫出可行高度再填 ✓")
        print("    graspable: ?        # 窄边 {:.3f} m ≤ 0.075 且几何自检有可行角 ⇒ true ✓".format(
            min(sz[0], sz[1])))
    if unknown:
        print("  ↑ 把上面几段补进 turtlebot3_manipulation_grasp/config/objects.yaml，")
        print("    再用 `python3 src/HANDOFF_harness/check_grasp_geometry.py <名称>` 定 grasp_lift ✓")

    print("\n── 还要同步的地方（换世界的完整清单）──────────────────────────")
    print("  1) launch（已参数化 ✓）：")
    print("       ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py \\")
    print("            world:={} spawn_x:=<出生x> spawn_y:=<出生y> spawn_yaw:=<出生yaw>".format(
        os.path.abspath(world_path)))
    print("  2) AMCL 初始位姿：grasp_phase.py 的 INITIAL_X, INITIAL_Y, INITIAL_YAW")
    print("       必须与上面 spawn_* 一致（不一致 ⇒ 一启动定位就偏 ✗）")
    print("  3) 抓哪张桌子：见上面打印的 support_surface.* 三行（改 grasp_params.yaml ✓）")
    if tables:
        nb = ", ".join("({:g}, {:g})".format(t.pose[0], t.pose[1]) for (t, _s) in tables[:4])
        print("  4) 邻居桌（只影响「隔着本桌看到的物体算不算目标」）：")
        print("       grasp_phase.py 的 NEIGHBOR_TABLES = ({})   # 及 NEIGHBOR_HALF".format(nb))
    print("  5) 地图：不用跑 SLAM，直接从 world 生成 ✓")
    print("       python3 HANDOFF_harness/world_tools.py map {}".format(os.path.abspath(world_path)))
    print("     然后按它打印的 cp 命令覆盖 nav2 的 map.pgm/yaml ✓")
    print("  6) 物体目录：上面每个物体都要在 objects.yaml 里有条目（含 graspable ✓）")
    if meshy:
        print("\n⚠ 这些模型只有网格碰撞（地图会跳过它们，请确认是否需要）：{}".format(meshy[:6]))
    print("=" * 92)


# ══════════════════ 子命令 ②：map ══════════════════
def cmd_map(world_path, outdir=None, resolution=0.05, pad=0.6):
    keys = load_object_keys()
    entries, bbox, missing = read_world(world_path)
    if missing:
        print("⚠ {} 个 model:// 解析不到 ⇒ 它们不会被画进地图 ✗".format(len(missing)))
    # 收集障碍盒（排除"物体"模型：它们放在桌上，不该进地图 ✓）
    obst = []
    for e in entries:
        if not e.static:
            continue
        if keys.get(norm(e.name)) or keys.get(norm(os.path.basename(e.uri.rstrip("/")))):
            continue
        for (rel, sz) in e.boxes:
            if rel[2] + sz[2] / 2 < 0.05 or rel[2] - sz[2] / 2 > 1.6:
                continue                      # 太低/太高：车不会碰（桌面板 0.765 ✓ 在范围内 ✓）
            c, s = math.cos(e.pose[5]), math.sin(e.pose[5])
            wx = e.pose[0] + c * rel[0] - s * rel[1]
            wy = e.pose[1] + s * rel[0] + c * rel[1]
            obst.append((wx, wy, e.pose[5], sz[0], sz[1]))
    if not obst:
        print("✗ 没收集到任何障碍盒（这个 world 只有网格碰撞？）⇒ 地图会是空的 ✗")
        return 1
    xs = [p[0] for p in obst]
    ys = [p[1] for p in obst]
    x0, y0 = min(xs) - pad, min(ys) - pad
    x1, y1 = max(xs) + pad, max(ys) + pad
    w = int(math.ceil((x1 - x0) / resolution))
    h = int(math.ceil((y1 - y0) / resolution))
    grid = bytearray(b"\xfe" * (w * h))                    # 254 = free
    for (cx, cy, yaw, sx, sy) in obst:
        cs = box_corners_xy(cx, cy, yaw, sx, sy)
        minx = min(p[0] for p in cs); maxx = max(p[0] for p in cs)
        miny = min(p[1] for p in cs); maxy = max(p[1] for p in cs)
        i0 = max(0, int((minx - x0) / resolution))
        i1 = min(w - 1, int((maxx - x0) / resolution) + 1)
        j0 = max(0, int((miny - y0) / resolution))
        j1 = min(h - 1, int((maxy - y0) / resolution) + 1)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                px = x0 + (i + 0.5) * resolution
                py = y0 + (j + 0.5) * resolution
                c, s = math.cos(-yaw), math.sin(-yaw)
                dx, dy = px - cx, py - cy
                lx, ly = c * dx - s * dy, s * dx + c * dy
                if abs(lx) <= sx / 2 and abs(ly) <= sy / 2:
                    grid[(h - 1 - j) * w + i] = 0          # PGM 行从上往下 = y 从大到小 ✓
    out = outdir or os.path.join(os.path.dirname(os.path.abspath(world_path)), "map_from_world")
    os.makedirs(out, exist_ok=True)
    pgm = os.path.join(out, "map.pgm")
    with open(pgm, "wb") as f:
        f.write("P5\n{} {}\n255\n".format(w, h).encode())
        f.write(bytes(grid))
    yml = os.path.join(out, "map.yaml")
    with open(yml, "w") as f:
        f.write("image: map.pgm\nmode: trinary\nresolution: {}\n".format(resolution))
        f.write("origin: [{:.2f}, {:.2f}, 0]\n".format(x0, y0))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    occ = sum(1 for b in grid if b == 0)
    print("=" * 92)
    print("已生成地图（从 world 直接栅格化，不用跑 SLAM ✓）")
    print("  {}".format(pgm))
    print("  {}   ({:.2f} × {:.2f} m @ {} m/格，占用格 {} = {:.1f}%)".format(
        yml, w * resolution, h * resolution, resolution, occ, 100.0 * occ / len(grid)))
    print("\n⚠ 什么时候**不需要**换图：新 world 与旧 world 是【同一个场地】（桌子/墙位置没变）"
          "⇒ 现用 map.pgm 仍然对得上，直接沿用 ✓")
    print("   （判据：两边的桌子位姿与家具模型逐字节一致；本工具生成的图与现用图只在"
          "SLAM 的未知区/薄墙边界上差 1~2 格 ✓）")
    nav = os.path.join(SRC, "turtlebot3_manipulation_navigation2", "map")
    print("\n替换 nav2 地图（换世界后旧图作废 ✓，建议先备份）：")
    print("    cp {}/map.pgm {}/map.pgm.bak_$(date +%H%M) 2>/dev/null".format(nav, nav))
    print("    cp {} {}/map.pgm".format(pgm, nav))
    print("    cp {} {}/map.yaml".format(yml, nav))
    print("  然后重启仿真与 nav2（AMCL 会按新图定位 ✓）")
    print("=" * 92)
    return 0


# ══════════════════ 子命令 ③：status ══════════════════
def cmd_status(world_arg="example.world"):
    """一条命令看清当前状态（换世界出问题时先跑它 ✓）

    打印：worlds 目录里有哪些文件 / launch 默认值会解析到哪个文件（含 md5、世界名）/
    该世界的缺模型与桌子/物体概览 / 当前 grasp_params.yaml 的 support_surface /
    当前 count_items.yaml 的待计数清单 ✓
    """
    import hashlib
    src_worlds = os.path.join(SRC, "wpr_simulation_ros2", "worlds")
    share = None
    try:
        import subprocess
        share = subprocess.run(["ros2", "pkg", "prefix", "wpr_simulation_ros2"],
                               capture_output=True, text=True, timeout=15).stdout.strip()
        share = os.path.join(share, "share", "wpr_simulation_ros2") if share else None
    except Exception:
        share = None
    install_worlds = os.path.join(share, "worlds") if share else None

    print("=" * 92)
    print("① worlds 目录")
    for tag, d in (("源码树", src_worlds), ("install", install_worlds)):
        if not d or not os.path.isdir(d):
            print("  {:<8} {}".format(tag, d or "（拿不到）"))
            continue
        print("  {:<8} {}".format(tag, d))
        for f in sorted(os.listdir(d)):
            if f.endswith(".world"):
                fp = os.path.join(d, f)
                real = os.path.realpath(fp)
                md = hashlib.md5(open(fp, "rb").read()).hexdigest()[:8] if os.path.isfile(fp) else "?"
                print("      {:<34} {}  {}  {}".format(
                    f, "软链→" + os.path.basename(real) if os.path.islink(fp) else "普通文件",
                    md, "" if os.path.isfile(fp) else "✗ 打不开"))
    print("\n② launch 默认值 {} 会解析到哪个文件".format(world_arg))
    resolved, cands = None, []
    if install_worlds:
        resolved, cands = _resolve_for_status(world_arg, install_worlds)
    if resolved:
        md = hashlib.md5(open(resolved, "rb").read()).hexdigest()[:8]
        try:
            root = ET.parse(resolved).getroot()
            w = root if root.tag == "world" else root.find("world")
            nm = w.get("name")
        except Exception:
            nm = "（解析失败）"
        print("  ✓ {}".format(resolved))
        print("    世界名 = {} ｜ md5 {} ｜ {:.1f} KB".format(
            nm, md, os.path.getsize(resolved) / 1024.0))
    else:
        print("  ✗ 找不到 {} —— 候选：".format(world_arg))
        for c in cands:
            print("      {}".format(c))
        print("    ⇒ 启动时用 world:=<文件名或绝对路径> 指定一个存在的 ✓")
    if resolved:
        print("\n③ 该世界的内容（缺模型/桌子/物体）")
        cmd_check(resolved)
    print("\n④ 当前抓取配置 grasp_params.yaml 的 support_surface")
    gp = os.path.join(SRC, "turtlebot3_manipulation_grasp", "config", "grasp_params.yaml")
    try:
        import yaml
        n = ((yaml.safe_load(open(gp, encoding="utf-8")) or {}).get("/**") or {}).get("ros__parameters") or {}
        pose = n.get("support_surface.pose")
        print("  pose = {}  length/width = {}/{}".format(
            pose, n.get("support_surface.length"), n.get("support_surface.width")))
        hit = None
        if resolved and pose:
            entries, _, _ = read_world(resolved)
            for e in entries:
                sl = e.top_slab
                if sl and abs(sl[0] - pose[0]) < 0.05 and abs(sl[1] - pose[1]) < 0.05:
                    hit = e.name
        print("  → 对应该世界里的 {}".format(hit if hit else "✗ 没对上任何一张桌子（要改）"))
    except Exception as e:                                # noqa: BLE001
        print("  （读取失败: {}）".format(e))
    print("\n⑤ 当前待计数清单 count_items.yaml")
    ci = os.path.join(SRC, "turtlebot3_manipulation_navigation2", "config", "count_items.yaml")
    try:
        import yaml
        d = yaml.safe_load(open(ci, encoding="utf-8")) or {}
        print("  count_items   = {}".format([x.get("name") for x in (d.get("count_items") or [])]))
        print("  answer_classes= {}   group_number = {}".format(
            d.get("answer_classes"), d.get("group_number")))
    except Exception as e:                                # noqa: BLE001
        print("  （读取失败: {}）".format(e))
    print("=" * 92)
    return 0


def _resolve_for_status(world_arg, install_worlds):
    cands = []
    base = os.path.basename(world_arg or "")
    if world_arg:
        cands.append(world_arg)
        if base:
            cands.append(os.path.join(install_worlds, base))
            link = os.path.join(install_worlds, "example.world")
            if os.path.islink(link):
                cands.append(os.path.join(os.path.dirname(os.path.realpath(link)), base))
    for c in cands:
        if c and os.path.isfile(c):
            return c, cands
    return None, cands


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("check", "map", "status"):
        print(__doc__)
        return 2
    if sys.argv[1] == "status":
        return cmd_status(sys.argv[2] if len(sys.argv) > 2 else "example.world")
    if len(sys.argv) < 3:
        print("用法: world_tools.py check <world> | map <world> [outdir] | status [world名]")
        return 2
    if sys.argv[1] == "check":
        cmd_check(sys.argv[2])
        return 0
    outdir = sys.argv[3] if len(sys.argv) > 3 else None
    return cmd_map(sys.argv[2], outdir)


if __name__ == "__main__":
    sys.exit(main())
