#!/usr/bin/env python3
"""
视觉适配层 —— 把队友的 vision_pipeline 检测结果变成抓取侧的 GraspTargetStamped。

它实现的是**已经冻结的契约**（逐条对着 msg/GraspTargetStamped.msg 与
srv/DetectGraspTarget.srv 写，不改视觉侧任何代码）：

    /detect_grasp_target  (DetectGraspTarget)
        请求: class_ids[]        —— 想找哪几类；空数组 = 不限，全都要
        响应: success/message/targets[]（按置信度从高到低）

处理链（全部在本节点内完成）：
    1. 取一帧 RGB（/camera/image_raw）+ 对齐深度（/camera/depth/image_raw）
       + 内参（/camera/camera_info）—— launch 已把三者 frame_id 统一成
       camera_rgb_optical_frame，像素一一对应
    2. vision_pipeline.VisionPipeline().detect(bgr) → 框 + 掩码 + 掩码质心
       classify_phrase(phrase) 归一到 ITEM_NAMES；不在词表里的丢掉
    3. 掩码 + 深度 → 相机系点云 → TF(camera_rgb_optical_frame → base_footprint)
       → 得到物体【可见表面】的质心（机器人内部固定变换，不含 AMCL 误差）
    4. 换算成契约要求的点：**物体中心轴 ∩ 支撑面**（z = 支撑面高度）。
       本机位下【不能】用契约里那句"视线与支撑面求交" —— 相机装在 z=0.80 m、
       几乎与桌面(0.78 m)等高，穿过物体中心的视线是水平甚至向上(+6.9°)的，
       与支撑面没有前向交点 ✗。改用等价的、在这个机位下成立的算法：
           可见表面质心 + 沿【水平视线方向】后退 半个物体厚度（查 objects.yaml）
       —— 可见表面本来就比物体轴心近半个厚度，退回来就落在轴心上 ✓
       同时另算一个"掩码底部接触带"的版本，两个都打日志。
    5. confidence = 检测分数；stamp = 本次检测时刻；frame_id = base_footprint

运行环境：**必须**在装了 torch/groundingdino/sam2 的虚拟环境里跑，例如
    source ~/venvs/vision_env/bin/activate
    ros2 run turtlebot3_manipulation_grasp detect_grasp_target_node.py
自测（不用编排节点，直接抓一帧打结果）：
    ros2 run turtlebot3_manipulation_grasp detect_grasp_target_node.py --self-test
"""

import math
import os
import re
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

import tf2_ros
from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
from turtlebot3_manipulation_grasp.srv import DetectGraspTarget

# ═══════════════════════════════════════════════════════════════
# 定位队友的 vision_pipeline（不 import 他们的仓库到我们包里）
# 安装后它和本脚本不在同一个目录：nav2 包的脚本装在
#   <prefix>/lib/turtlebot3_manipulation_navigation2/
# ═══════════════════════════════════════════════════════════════
_NAV2_PKG = "turtlebot3_manipulation_navigation2"


def find_nav2_scripts_dir():
    """返回装着 vision_pipeline.py 的目录（找不到返回 None）。"""
    cands = []
    try:                                     # 首选 ament 索引（若 venv 里装了 ament_index_python）
        from ament_index_python.packages import get_package_prefix
        cands.append(os.path.join(get_package_prefix(_NAV2_PKG), "lib", _NAV2_PKG))
    except Exception:
        pass
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if prefix:
            cands.append(os.path.join(prefix, "lib", _NAV2_PKG))
            cands.append(os.path.join(prefix, "share", _NAV2_PKG, "scripts"))
    for c in cands:
        if c and os.path.isfile(os.path.join(c, "vision_pipeline.py")):
            return c
    return None


# ═══════════════════════════════════════════════════════════════
# 抓取包的配置：类别尺寸（objects.yaml）+ 支撑面高度（grasp_params.yaml）
# ═══════════════════════════════════════════════════════════════
def _find_pkg_config(name):
    """在 <prefix>/share/<pkg>/config/ 下找 name（找不到返回 None）。"""
    here = os.path.dirname(os.path.realpath(__file__))
    # 源码树里直接跑的情况：<pkg>/scripts/x.py → <pkg>/config/
    src = os.path.join(os.path.dirname(here), "config", name)
    if os.path.isfile(src):
        return src
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        p = os.path.join(prefix, "share", "turtlebot3_manipulation_grasp", "config", name)
        if os.path.isfile(p):
            return p
    return None


def load_catalog(catalog_path):
    """objects.yaml → {class_id: {"dims": (d,w,h), "graspable": bool}}（启用的类别）。"""
    out = {}
    if not catalog_path or not os.path.isfile(catalog_path):
        return out
    try:
        import yaml
        doc = yaml.safe_load(open(catalog_path, encoding="utf-8")) or {}
        for cls, v in (doc.get("objects") or {}).items():
            if isinstance(v, dict):
                out[cls] = {"dims": (float(v.get("depth", 0.0)), float(v.get("width", 0.0)),
                                     float(v.get("height", 0.0))),
                            "graspable": bool(v.get("graspable", False))}
    except Exception:
        pass
    return out


def load_object_sizes(catalog_path):        # 兼容旧调用
    return {k: v["dims"] for k, v in load_catalog(catalog_path).items()}


def support_surface_z_from_params(params_path, default=0.78):
    """grasp_params.yaml 的 support_surface（pose z + thickness/2 = 桌面顶）→ 高度。

    与抓取侧共用同一份配置，避免两边各写一个数 ✗
    """
    if not params_path or not os.path.isfile(params_path):
        return default
    try:
        import yaml
        doc = yaml.safe_load(open(params_path, encoding="utf-8")) or {}
        rp = (doc.get("/**") or {}).get("ros__parameters") or doc.get("ros__parameters") or {}
        pose = rp.get("support_surface.pose")
        thick = float(rp.get("support_surface.thickness", 0.0))
        if pose and len(pose) >= 3:
            return float(pose[2]) + thick / 2.0
    except Exception:
        pass
    return default


def _yaw_from_params(params_path, default=0.0):
    """grasp_params.yaml 的 object_yaw_map（物体在 map 里的偏航，本世界全轴对齐 ⇒ 0）。"""
    if not params_path or not os.path.isfile(params_path):
        return default
    try:
        import yaml
        doc = yaml.safe_load(open(params_path, encoding="utf-8")) or {}
        rp = (doc.get("/**") or {}).get("ros__parameters") or doc.get("ros__parameters") or {}
        v = rp.get("object_yaw_map")
        return float(v) if v is not None else default
    except Exception:
        return default


class DetectGraspTargetNode(Node):
    """提供 /detect_grasp_target 服务：检测 + 3D 定位 + 换算成契约点。"""

    def __init__(self):
        super().__init__("detect_grasp_target")

        # ── 参数 ────────────────────────────────────────────────
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("camera_frame", "camera_rgb_optical_frame")
        self.declare_parameter("support_surface_z", -1.0)   # <0 = 从 grasp_params.yaml 读
        self.declare_parameter("stream_timeout", 5.0)       # s，等服务被调用时凑齐三路数据
        self.declare_parameter("max_range", 2.0)            # m，超过就认为深度不可信
        # ── 抓取模式：按 objects.yaml 全类别识别（闭集复核照开，见 _detect_pass）──
        # 计数任务的词表只有 4 类（apple/coke can/bowl/banana），可抓的物体常常不在里面
        # （例如餐桌上的 sugar_box）→ 那 4 类里只认得夹不住的碗 ✗
        # auto：先按队友默认（4 类 + 复核）跑一次；结果里【没有可抓类别】时，
        #       再用"全类别 + 跳过复核"跑第二次（召回优先）
        # always / never 可强制。跳过复核的代价见文件末尾说明。
        self.declare_parameter("grasp_mode", "auto")        # auto | always | never
        self.declare_parameter("size_check", True)          # 用实测尺寸核对类别（防误标）
        self.declare_parameter("size_ratio_lo", 0.7)        # 实测/期望 下限
        self.declare_parameter("size_ratio_hi", 1.4)        # 实测/期望 上限（按对角线算）
        # ★ 高度核对（掩码点云在竖直方向的跨度 vs 目录表高度）：这是"抓空气"的唯一防线 ——
        #   标签高度错了，抓取侧就会按错误的碰撞箱/抓取高度规划：
        #   实测把 0.035 m 高的 pudding_box 认成 0.25 m 高的 chips_can →
        #   手指在物体上方 10 cm 合拢，而 stage 仍然报 success ✗
        self.declare_parameter("height_check", True)
        self.declare_parameter("height_ratio_lo", 0.55)      # 实测/期望 下限（掩码缺边时留余量）
        self.declare_parameter("height_ratio_hi", 1.80)      # 上限
        # 多帧投票：连续取 N 帧各检一次，某类别要在 ≥⌈2N/3⌉ 帧出现才采纳（置信度取中位数）
        # 为什么：假阳性是逐帧随机的（这帧冒 tuna fish can，下帧就没了），真目标稳定出现 ✓
        # 代价：每帧 ~4~9 s，N=3 大约多花 8~18 s
        self.declare_parameter("vote_frames", 3)
        # ── ★ 位置补丁（2026-09-15）：位置估计的三处改动 ────────────
        # 现场症状：糖盒/番茄罐都"夹空"，实测合爪间距=指令值 ⇒ 两指之间没有东西；
        # 反推真值必须偏 ≥44 mm（径向）/≥60 mm（横向）才能夹空，而两条独立估计
        # （掩码法/框心法）彼此只差 1~15 mm ⇒ 是**共同偏置**，不是随机噪声。
        # 三个补丁，各自独立、可单独关掉做 A/B：
        #  ① position_source=plane：改用【地面约束测距】（掩码底边 + 已知桌面高度，
        #     完全不用深度图）——与深度链路独立，天然躲开"掩码吃进桌面/背景导致偏远"
        #  ② mask_erode_px：掩码先腐蚀几像素，躲开彩色/深度不对齐的毛边
        #  ③ 距离闸门（在 _mask_points_cam 里，无需参数）：只留物体自己那一簇深度
        self.declare_parameter("position_source", "auto")   # auto | plane | mask
        self.declare_parameter("mask_erode_px", 2)
        self.declare_parameter("plane_min_den", 0.10)       # 平面约束条件数下限
        self.declare_parameter("yaw_backoff", True)         # 长方体按支撑函数后退
        self.declare_parameter("table_check", True)         # 每次检测顺带校验桌平面
        self.declare_parameter("map_frame", "map")
        # 评分要求"在仿真相机画面或等效可视化里框出拟抓取目标、标注物品名称" →
        # 每次检测把带框+类别文字的图发出来并存盘，作为可核查的证据 ✓
        self.declare_parameter("annotated_topic", "/detect_grasp_target/annotated_image")
        self.declare_parameter("save_annotated", True)
        self.declare_parameter("annotated_dir", os.path.expanduser("~/turtlebot3_detections"))

        p = lambda n: self.get_parameter(n).value
        self.target_frame = p("target_frame")
        self.camera_frame = p("camera_frame")
        self.stream_timeout = float(p("stream_timeout"))
        self.max_range = float(p("max_range"))

        cfg_objects = _find_pkg_config("objects.yaml")
        cfg_params = _find_pkg_config("grasp_params.yaml")
        self.catalog = load_catalog(cfg_objects)
        self.object_sizes = {k: v["dims"] for k, v in self.catalog.items()}
        self.grasp_mode = str(p("grasp_mode") or "auto").lower()
        self.size_check = bool(p("size_check"))
        self.height_check = bool(p("height_check"))
        self.height_lo = float(p("height_ratio_lo"))
        self.height_hi = float(p("height_ratio_hi"))
        self.size_lo = float(p("size_ratio_lo"))
        self.size_hi = float(p("size_ratio_hi"))
        self.vote_frames = max(1, int(p("vote_frames")))
        self.position_source = str(p("position_source") or "auto").lower()
        self.mask_erode_px = max(0, int(p("mask_erode_px")))
        self.plane_min_den = float(p("plane_min_den"))
        self.yaw_backoff = bool(p("yaw_backoff"))
        self.table_check = bool(p("table_check"))
        self.map_frame = str(p("map_frame") or "map")
        self.object_yaw_map = _yaw_from_params(cfg_params)
        # 抓取模式的 prompt：objects.yaml 的全部类别名（下划线换成空格，GroundingDINO 好认）
        self.grasp_prompt = " . ".join(c.replace("_", " ") for c in sorted(self.catalog)) + " ."
        self.get_logger().info("抓取模式={} 全类别 prompt={}".format(
            self.grasp_mode, self.grasp_prompt[:90]))
        sup = p("support_surface_z")
        self.support_z = float(sup) if sup is not None and float(sup) >= 0.0 \
            else support_surface_z_from_params(cfg_params)
        self.get_logger().info(
            "支撑面高度 = {:.3f} m（来自 {}）；objects.yaml = {}（{} 类）".format(
                self.support_z, "参数" if (sup is not None and float(sup) >= 0.0) else "grasp_params.yaml",
                cfg_objects, len(self.object_sizes)))

        # ── 视觉流水线（懒加载 + **后台预热**）───────────────────
        # ★ 带上 INSID3 闭集复核后，构造一次要 ~77 s（DINOv3 骨干 + 原型预计算），
        #   远大于没有复核时的 ~4 s ✗。所以启动后立刻在后台线程里加载，
        #   不要等第一次服务调用才卡 77 s（那时机器人已经站在餐桌旁干等了）。
        self._vp = None
        self._vp_mod = None            # vision_pipeline 模块本身（模块级函数在这里）
        self._insid3 = None            # insid3_review 模块（抓取模式要换它的类别集合）
        self._vp_error = None
        self._vp_lock = threading.Lock()
        self._nav2_scripts = find_nav2_scripts_dir()
        if self._nav2_scripts is None:
            self.get_logger().error(
                "找不到 vision_pipeline.py（查过 AMENT_PREFIX_PATH 里的 "
                "$(prefix)/lib/turtlebot3_manipulation_navigation2）——先 source 工作区的 install/setup.bash")

        # ── 数据缓存：三路话题各留最新一帧 ──────────────────────
        self._lock = threading.Lock()
        self._img = None
        self._depth = None
        self._info = None
        self.create_subscription(Image, p("image_topic"), self._on_image, 1)
        self.create_subscription(Image, p("depth_topic"), self._on_depth, 1)
        self.create_subscription(CameraInfo, p("camera_info_topic"), self._on_info, 1)

        # ── TF ──────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── 标注图（评分要求：框出拟抓取目标 + 标注物品名称）────────
        self.annotated_topic = p("annotated_topic")
        self.save_annotated = bool(p("save_annotated"))
        self.annotated_dir = p("annotated_dir")
        self._ann_pub = None
        if self.save_annotated:
            try:
                os.makedirs(self.annotated_dir, exist_ok=True)
            except Exception as e:
                self.get_logger().warn("标注图目录建不出来（{}），改为只发话题".format(e))
                self.save_annotated = False
        if self.save_annotated or self.annotated_topic:
            from sensor_msgs.msg import Image as _Img
            self._ann_pub = self.create_publisher(_Img, self.annotated_topic, 1)

        self._srv = self.create_service(
            DetectGraspTarget, "/detect_grasp_target", self.handle_detect)
        self.get_logger().info(
            "就绪：/detect_grasp_target（相机 {} → 目标帧 {}）；标注图 → {} {}"
            .format(self.camera_frame, self.target_frame, self.annotated_topic,
                    "（并存盘到 {}）".format(self.annotated_dir) if self.save_annotated else ""))

        # 后台预热：启动就把权重读进来（~77 s），机器人导航过来这段时间正好用掉 ✓
        threading.Thread(target=self._warmup, daemon=True).start()

    def _warmup(self):
        try:
            self.get_logger().info("后台预热视觉流水线（含 INSID3 复核，约 1 分钟）...")
            ok = self._ensure_pipeline()
            self.get_logger().info("预热完成：{}".format(
                "流水线就绪 ✓" if ok else "失败（服务调用时会再试）: {}".format(self._vp_error)))
        except Exception as e:                 # noqa: BLE001
            self.get_logger().warn("预热异常（不影响服务，调用时会重试）: {}".format(e))

    # ── 标注图：把框 + 类别 + 算出来的轴心点画上去 ───────────────
    def _publish_annotated(self, bgr, items, stamp):
        """items: [(class_id, score, box_xyxy, point_xyz)]。存盘 + 发话题。"""
        if not items or (self._ann_pub is None and not self.save_annotated):
            return
        try:
            import cv2
        except Exception:
            return
        img = bgr.copy()
        for cls, score, box, pt in items:
            x1, y1, x2, y2 = [int(round(v)) for v in box]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, "{} {:.2f}".format(cls, score), (x1, max(12, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(img, "({:.3f},{:.3f})".format(pt[0], pt[1]), (x1, min(img.shape[0] - 4, y2 + 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1, cv2.LINE_AA)
        if self._ann_pub is not None:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.camera_frame
            msg.height, msg.width = img.shape[0], img.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = img.shape[1] * 3
            msg.data = img.tobytes()
            self._ann_pub.publish(msg)
        if self.save_annotated:
            ok, buf = cv2.imencode(".png", img)
            if ok:
                path = os.path.join(self.annotated_dir,
                                    "detect_{:.3f}.png".format(stamp.sec + stamp.nanosec * 1e-9))
                try:
                    open(path, "wb").write(buf.tobytes())
                    self.get_logger().info("  标注图已存: {}".format(path))
                except Exception as e:
                    self.get_logger().warn("标注图存盘失败: {}".format(e))

    # ── 话题回调 ───────────────────────────────────────────────
    def _on_image(self, msg):
        with self._lock:
            self._img = msg

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def _on_info(self, msg):
        with self._lock:
            self._info = msg
            if msg.header.frame_id:
                self.camera_frame = msg.header.frame_id   # launch 里已统一成光学帧

    def _spin_for(self, seconds):
        """保持 spin 地等（use_sim_time 下时钟只在 spin 时前进）。

        ★ 只能在【服务回调之外】用（例如自测）。服务回调里不能嵌套 spin ✗
          —— 回调是 executor 的线程在跑，再 spin 会死锁；服务回调里改成
          _wait_streams() 的"轮询 + 新鲜度判断"，靠 MultiThreadedExecutor
          让订阅回调在别的线程继续收数据。
        """
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _fresh_enough(self, msg, max_age=2.0):
        """消息是否够新（仿真钟或墙钟都按同一把尺子比）。"""
        if msg is None:
            return False
        try:
            stamp = Time.from_msg(msg.header.stamp)
            now = self.get_clock().now()
            age = abs((now - stamp).nanoseconds) / 1e9
            return age <= max_age
        except Exception:
            return True          # 拿不到时间戳就不卡这条

    def _detect_pass(self, bgr, prompt=None, use_reviewer=True):
        """跑一次检测。

        prompt=None 用队友默认（4 类词表）；use_reviewer=False 时**跳过 INSID3 闭集复核**。

        ★ 抓取模式（prompt 非空）时会把 insid3_review.TARGET_CLASSES 临时换成
          objects.yaml 的全部类别：队友那份 `review()` 里有
          `if label not in TARGET_CLASSES: continue`，只认 4 个目标类 →
          其余 14 类（糖盒、芥末瓶…）会被**全部丢掉** ✗，所以我们以前在抓取模式
          关掉了复核 —— 代价是标签退回 GroundingDINO 的自由文本 phrase，
          实测逐帧在 mustard_bottle / tomato_soup_can / cracker_box / chips_can
          之间乱跳、位置能差 1 m ✗。现在改成"换类别集合、复核照开"：
          闭集 argmax（DINOv3 去偏置特征 × 参考原型余弦）给标签，
          GroundingDINO 只负责"哪里可能有东西" ✓ 不改队友文件本身（只换运行时常量）。
        这两件事都只是改 VisionPipeline 实例属性，**不动队友的代码** ✓
        （detect() 逐帧读 self.text_prompt，并按 self.reviewer is not None 决定要不要复核）
        """
        vp = self._vp
        old_prompt, old_rev = vp.text_prompt, vp.reviewer
        old_ids = getattr(vp, "_input_ids", None)

        def _refresh_ids(pr):
            """★ 换 prompt 必须重算分词缓存：detect() 的短语解码读的是 self._input_ids ✗
            （不重算就会用旧词表的 token 去解码新词表的框 → 出现"只填 sugar box 却报 apple"这种鬼结果）
            """
            try:
                tok = vp.gdino_model.tokenizer([pr], padding="longest", return_tensors="pt")
                vp._input_ids = tok["input_ids"][0].tolist()
            except Exception as e:
                self.get_logger().warn("重算分词缓存失败: {}".format(e))

        old_tc = None
        try:
            if prompt:
                vp.text_prompt = prompt
                _refresh_ids(prompt)
                if self._insid3 is not None and use_reviewer:
                    # ★ 必须用【复核模块自己的类别名】而不是 objects.yaml 的 key ✗
                    #   复核返回的 label 取自 INSID3_CLASSES（"sugar box" 带空格），
                    #   而 objects.yaml 的 key 是 "sugar_box"（下划线）→ 直接拿 key 当
                    #   TARGET_CLASSES 会让 review() 里的 `label not in TARGET_CLASSES`
                    #   把**所有**框都丢掉（实测：抓取模式 3 帧全 {}）✗
                    #   统一用复核的标签集，之后由 _classify_catalog 归一化（空格↔下划线）✓
                    old_tc = getattr(self._insid3, "TARGET_CLASSES", None)
                    self._insid3.TARGET_CLASSES = list(self._insid3.INSID3_CLASSES.keys())
            vp.reviewer = old_rev if use_reviewer else None
            return vp.detect(bgr)
        finally:
            if old_tc is not None:
                self._insid3.TARGET_CLASSES = old_tc
            vp.text_prompt, vp.reviewer = old_prompt, old_rev
            if old_ids is not None:
                vp._input_ids = old_ids

    def _classify_catalog(self, phrase):
        """把 phrase 归一到 objects.yaml 的类别名（跳过复核、用自己的词表时用）。"""
        def norm(x):
            return re.sub(r"[^a-z0-9]+", " ", (x or "").lower()).strip()
        pz = norm(phrase)
        if not pz:
            return None
        for cls in self.catalog:
            if norm(cls) == pz:
                return cls
        words = pz.split()
        for cls in self.catalog:                      # 退一步：类别词全被包含
            cw = norm(cls).split()
            if cw and all(w in words for w in cw):
                return cls
        return None

    def _size_ok(self, cls, pts_cam):
        """实测尺寸核对类别：掩码点云的切向跨度应落在 [窄边, 对角线] 附近。

        为什么需要：跳过复核后开集标签会错（实测 apple 被标成 coke can 0.602 ✗）。
        类的尺寸差得远时（碗 0.16 被标成罐 0.067）这一步就能拦掉 ✗
        返回 (ok, 实测跨度, 期望区间)。
        """
        if not self.size_check or cls not in self.object_sizes or pts_cam is None:
            return True, None, None
        d, w, h = self.object_sizes[cls]
        if d <= 0 or w <= 0:
            return True, None, None
        # 用 2%~98% 分位而不是 max-min：同样是为了不被掩码边缘/桌面污染放大 ✗
        lo, hi = np.percentile(pts_cam[:, 0], [2, 98])
        obs = float(hi - lo)                                             # 相机系 x = 横向
        narrow, diag = min(d, w), math.hypot(d, w)
        ok = (narrow * self.size_lo) <= obs <= (diag * self.size_hi)
        return ok, obs, (narrow * self.size_lo, diag * self.size_hi)

    @staticmethod
    def _to_target_frame(pts_cam, tf):
        """相机光学系点云 → 目标帧（用 TF 的四元数/位移，与 _contract_point 同一套）。"""
        q, t = tf.transform.rotation, tf.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        return pts_cam.dot(R.T) + np.array([t.x, t.y, t.z])

    def _height_ok(self, cls, pts_cam, tf):
        """用掩码点云【竖直方向的跨度】核对标签高度；返回 (ok, 实测高度, 期望区间)。

        为什么必须有这道校验（实测故障，2026-09-14）：
            开集/闭集标签都可能把物体认成"另一个高矮差很多"的类。抓取侧的碰撞箱尺寸、
            抓取高度全都按**类别**查 objects.yaml → 高度一错就整段错：
            把 0.035 m 高的 pudding_box 认成 0.25 m 高的 chips_can 时，
            抓取高度变成 0.102 m → 手指在物体上方 10 cm 合拢，
            而 MTC 仍然"规划成功、执行成功"（attach 是逻辑附着）→ 表现成"抓了个寂寞" ✗

        做法：掩码像素 + 深度反投影成相机系点云（复用 _mask_points_cam），
        用 TF 变到目标帧（base_footprint，z 向上），取 z 的 2%~98% 分位跨度当实测高度。
        相机略微俯仰也不影响 —— 竖直方向取自目标帧，不是图像的行方向 ✓
        """
        if not self.height_check or cls not in self.object_sizes or pts_cam is None:
            return True, None, None
        _, _, h = self.object_sizes[cls]
        if h <= 0:
            return True, None, None
        pts_t = self._to_target_frame(pts_cam, tf)
        zs = pts_t[:, 2]
        z02, z98 = np.percentile(zs, 2), np.percentile(zs, 98)
        obs = float(z98 - z02)
        # 实测顶面高度（目标帧 z）—— 抓取侧"指尖平面高度"要和它比 ✓
        self.get_logger().info("  实测物体 z 范围 {:.3f}~{:.3f} m（顶面 {:.3f}，高度 {:.3f}）"
                               .format(z02, z98, z98, obs))
        ok = (h * self.height_lo) <= obs <= (h * self.height_hi)
        return ok, obs, (h * self.height_lo, h * self.height_hi)

    def _wait_streams(self, timeout):
        """等 RGB / 深度 / 内参三路都到位且新鲜。

        服务是按需调用（停稳后调一次），这里短暂轮询可接受；
        ★ 不 spin（见 _spin_for 的说明），靠 MultiThreadedExecutor 收数据。
        """
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            with self._lock:
                img, dep, info = self._img, self._depth, self._info
            if (self._fresh_enough(img) and self._fresh_enough(dep)
                    and self._fresh_enough(info)):
                return True
            time.sleep(0.05)
        return False

    # ── 图像解码（不用 cv_bridge：numpy 2.x 与 ROS 的 cv_bridge ABI 不兼容）──
    @staticmethod
    def _to_bgr(msg):
        h, w, enc = msg.height, msg.width, (msg.encoding or "").lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("rgb8", "bgr8"):
            img = data.reshape(h, w, 3)
            return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy()
        if enc in ("rgba8", "bgra8"):
            img = data.reshape(h, w, 4)[:, :, :3]
            return img[:, :, ::-1].copy() if enc == "rgba8" else img.copy()
        return None

    @staticmethod
    def _to_depth(msg):
        h, w = msg.height, msg.width
        data = np.frombuffer(msg.data, dtype=np.uint8)
        enc = (msg.encoding or "").lower()
        if enc == "32fc1":
            return data.view(np.float32).reshape(h, w).copy()
        if enc in ("16uc1", "mono16"):
            return data.view(np.uint16).reshape(h, w).astype(np.float32) / 1000.0
        return None

    # ── 视觉流水线 ─────────────────────────────────────────────
    def _ensure_pipeline(self):
        if self._vp is not None:
            return True
        if self._vp_error:
            return False
        # 预热线程可能正在加载 → 加锁，避免并发加载两次 ✗
        with self._vp_lock:
            if self._vp is not None:
                return True
            if self._vp_error:
                return False
            return self._load_pipeline_unlocked()

    def _load_pipeline_unlocked(self):
        if self._nav2_scripts is None:
            self._vp_error = "找不到 vision_pipeline.py（先 source 工作区 install/setup.bash）"
            return False
        if self._nav2_scripts not in sys.path:
            sys.path.insert(0, self._nav2_scripts)
        t0 = time.time()
        try:
            self.get_logger().info("首次调用：加载 GroundingDINO + SAM2（数十秒）...")
            import vision_pipeline as vp
            self._vp_mod = vp          # ★ classify_phrase 是模块级函数，不是类方法 ✗
            try:
                # 队友的闭集复核模块：抓取模式要临时换它模块级的 TARGET_CLASSES（见 _detect_pass）
                import insid3_review
                self._insid3 = insid3_review
            except Exception as e:                       # noqa: BLE001
                self._insid3 = None
                self.get_logger().warn("insid3_review 不可用（{}）→ 抓取模式只能用自由文本标签"
                                       .format(e))
            self._vp = vp.VisionPipeline()
            self.get_logger().info("视觉流水线就绪，耗时 {:.1f}s；词表 {}".format(
                time.time() - t0, vp.ITEM_NAMES))
            return True
        except Exception as e:
            self._vp_error = "{}: {}".format(type(e).__name__, e)
            self.get_logger().error(
                "视觉流水线加载失败：{}\n"
                "  多半是没在视觉虚拟环境里跑（torch/groundingdino/sam2 不在系统 python 里）✗\n"
                "  先：source ~/venvs/vision_env/bin/activate".format(self._vp_error))
            return False

    # ── TF / 几何 ──────────────────────────────────────────────
    def _lookup_cam_to_target(self):
        """相机光学帧 → 目标帧 的变换（机器人内部固定变换，含 AMCL 的链路不参与）。"""
        for src in (self.camera_frame, "camera_rgb_optical_frame",
                    "turtlebot3/camera_rgb_optical_frame"):
            for stamp in (Time(),):
                try:
                    return self._tf_buffer.lookup_transform(self.target_frame, src, stamp)
                except Exception:
                    continue
        return None

    @staticmethod
    def _rot_of(tf):
        """TF → 3×3 旋转矩阵（相机光学系 → target_frame）。"""
        q = tf.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])

    def _mask_points_cam(self, mask, depth, k, cls=None, patch=False):
        """掩码 + 深度 → 相机光学系点云（N×3），返回 (pts, ys, stats)。

        ★ 2026-09-15 补丁①【距离闸门】：只留"物体自己那一簇"深度。
          掩码里混进的桌面/背景都比物体**远** ⇒ 以 20% 分位（对少量近处毛刺免疫）
          为前缘，只保留 [前缘, 前缘 + 物体最大尺寸 + 3 cm] 以内的像素 ✓
          stats 回报剔除比例：剔得多 ⇒ 掩码确实在吃背景（"偏远"的直接证据 ✓）
        ★ 补丁②【腐蚀】：掩码先腐蚀几像素，躲开彩色/深度不对齐的毛边
        """
        if mask is None or depth is None:
            return None
        if mask.shape[:2] != depth.shape[:2]:
            return None
        stats = {}
        m = mask.astype(bool)
        # ★ patch=False ⇒ 完全走原逻辑（队友/计数任务那条路）：
        #   不腐蚀、不加距离闸门 ⇒ 返回的点云与补丁前逐字节一致 ✓
        ep = int(self.mask_erode_px) if patch else 0
        if ep > 0 and m.sum() > 60:
            try:
                import cv2
                kk = np.ones((2 * ep + 1, 2 * ep + 1), np.uint8)
                e = cv2.erode(m.astype(np.uint8), kk).astype(bool)
                if e.sum() >= max(20, 0.25 * m.sum()):
                    m = e
                    stats["erode"] = ep
            except Exception:                      # noqa: BLE001
                pass
        ys, xs = np.nonzero(m)
        if xs.size == 0:
            return None
        zs = depth[ys, xs].astype(float)
        ok = np.isfinite(zs) & (zs > 0.0) & (zs < self.max_range)
        if not np.any(ok):
            return None
        xs, ys, zs = xs[ok], ys[ok], zs[ok]
        n0 = int(zs.size)
        if patch and cls in self.object_sizes and n0 >= 40:
            d, w, h = self.object_sizes[cls]
            ext = max([x for x in (d, w, h) if x > 0] or [0.2])
            z_lo = float(np.percentile(zs, 20))
            keep = zs <= (z_lo + ext + 0.03)
            if int(keep.sum()) >= 20:
                stats["z_lo"] = z_lo
                stats["drop_far"] = 1.0 - float(keep.sum()) / float(n0)
                xs, ys, zs = xs[keep], ys[keep], zs[keep]
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        if fx <= 0 or fy <= 0:
            return None
        pts = np.stack([(xs - cx) * zs / fx, (ys - cy) * zs / fy, zs], axis=1)
        return pts, ys, stats

    def _plane_z_at(self, u, v, k, R, t_base):
        """像素 (u,v) 若落在支撑面 z=support_z 上，反解深度 Z（不用深度图）。

        相机系点 p = Z·((u-cx)/fx, (v-cy)/fy, 1)；base 系 z 分量 = R[2,:]·p + t_z
        令其 = support_z ⇒ Z = (support_z - t_z) / (R[2,:]·(...)) ✓
        返回 (Z | None, den)：|den| 太小 ⇒ 视线几乎与桌面平行，解极不稳定 ✗
        """
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        den = R[2, 0] * (u - cx) / fx + R[2, 1] * (v - cy) / fy + R[2, 2]
        if abs(den) < self.plane_min_den:
            return None, float(den)
        Z = (self.support_z - t_base[2]) / den
        if not (0.05 < Z < self.max_range):
            return None, float(den)
        return float(Z), float(den)

    def _backoff(self, cls, view_dir_base, tf):
        """可见面 → 物体轴心 的水平后退量（米）+ 依据说明。

        · 圆截面（d≈w）：任何水平视角下切平面都在轴外 r ⇒ 后退 r ✓
        · 长方体：后退量 = 支撑函数 (d/2)|cosφ| + (w/2)|sinφ|
          φ = 视线与物体 x 轴夹角（物体偏航 = object_yaw_map − map→base 的偏航）
          拿不到偏航时退化为圆周平均 (d+w)/π ✓（min(d,w)/2 是**下界**，
          斜看薄边时会少退 2 cm 以上 ✗）
        """
        d, w, h = self.object_sizes.get(cls, (0.0, 0.0, 0.0))
        if d <= 0 or w <= 0:
            return 0.0, "目录无尺寸"
        if abs(d - w) < 0.01:
            return 0.5 * max(d, w), "圆截面 r={:.3f}".format(0.5 * max(d, w))
        yaw_base = None
        try:
            tr = self._tf_buffer.lookup_transform(self.target_frame, self.map_frame, Time())
            q = tr.transform.rotation
            yaw_map_to_base = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                         1 - 2 * (q.y * q.y + q.z * q.z))
            yaw_base = self.object_yaw_map - yaw_map_to_base
        except Exception:                          # noqa: BLE001
            yaw_base = None
        if yaw_base is None or not self.yaw_backoff:
            return (d + w) / math.pi, "圆周平均 (d+w)/π"
        cphi = abs(math.cos(yaw_base) * view_dir_base[0] + math.sin(yaw_base) * view_dir_base[1])
        cphi = min(1.0, max(0.0, cphi))
        sphi = math.sqrt(max(0.0, 1.0 - cphi * cphi))
        return 0.5 * d * cphi + 0.5 * w * sphi, "支撑函数 φ={:.0f}°".format(
            math.degrees(math.acos(cphi)))

    def _ground_point(self, cls, mask, k, tf):
        """★ 补丁③【地面约束测距】：掩码底边 + 已知桌面高度 → 物体轴心点。

        完全不看深度图：物体"底边"像素落在桌面平面上（z=support_z 已知）
        ⇒ 由内参+相机外参反解该像素的深度，再反投影出底边前缘点，
        沿水平视线后退"可见面→轴心"的距离即轴心 ✓
        与深度链路独立 ⇒ 可交叉验证"位置偏远"到底出自深度还是掩码 ✗
        返回 (axis_base | None, info)
        """
        info = {}
        if mask is None or k is None or tf is None:
            return None, info
        m = mask.astype(bool)
        ys, xs = np.nonzero(m)
        if xs.size < 30:
            info["why"] = "掩码太小"
            return None, info
        u0, u1 = int(xs.min()), int(xs.max())
        wpx = u1 - u0 + 1
        lo, hi = u0 + int(0.25 * wpx), u0 + int(0.75 * wpx)
        if hi - lo < 3:
            lo, hi = u0, u1
        R = self._rot_of(tf)
        tr = tf.transform.translation
        t_base = np.array([tr.x, tr.y, tr.z])
        info["cam_h"] = float(t_base[2] - self.support_z)
        Zs, front = [], []
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        for u in range(lo, hi + 1):
            col = np.nonzero(m[:, u])[0]
            if col.size == 0:
                continue
            v = float(col.max())                   # 该列底边像素 = 物体与桌面接触处
            Z, _ = self._plane_z_at(u, v, k, R, t_base)
            if Z is None:
                continue
            pc = np.array([(u - cx) * Z / fx, (v - cy) * Z / fy, Z])
            front.append(R.dot(pc) + t_base)
            Zs.append(Z)
        if len(Zs) < 5:
            info["why"] = "底边可解像素不足({})".format(len(Zs))
            return None, info
        info["Z_med"] = float(np.median(Zs))
        info["jitter"] = float(np.percentile(Zs, 90) - np.percentile(Zs, 10))
        front_pts = np.stack(front, axis=0)         # 底边各列的前缘点（base 系，都在桌面上）
        c = front_pts.mean(axis=0)
        c[2] = t_base[2]
        v_ray = c - t_base
        v_ray[2] = 0.0
        n = float(np.linalg.norm(v_ray))
        if n < 1e-6:
            info["why"] = "视线退化"
            return None, info
        v_ray = v_ray / n
        s_hat = np.array([-v_ray[1], v_ray[0], 0.0])
        rel = front_pts - t_base
        rel[:, 2] = 0.0
        r_i = rel.dot(v_ray)                        # 沿视线距离
        s_i = rel.dot(s_hat)                        # 横向偏移
        # ★ 可见面 = 切平面 ⇒ 取沿视线的【近端分位】，不用中位数：
        #   斜看长方体时底边轮廓从"近角"拖到"远角"（跨度可达 9 cm），
        #   中位数落在物体中段 ⇒ 位置整体偏后 1 cm 以上 ✗
        r_f = float(np.percentile(r_i, 15))
        s_c = float(np.median(s_i))
        info["r_near"] = float(np.percentile(r_i, 5))
        info["jitter"] = float(np.percentile(r_i, 90) - np.percentile(r_i, 10))
        info["lat_half"] = 0.5 * float(np.percentile(s_i, 95) - np.percentile(s_i, 5))
        p_front = t_base + v_ray * r_f + s_hat * s_c
        p_front[2] = self.support_z
        info["front"] = p_front
        # ★ 置信判据：底边沿视线的跨度不能超过物体自身脚印对角线 + 5 cm
        #   （超了说明掩码吃进了桌面/背景——那正是"位置偏远"的可测特征 ✓）
        d0, w0, _h0 = self.object_sizes.get(cls, (0.0, 0.0, 0.0))
        diag = math.hypot(d0, w0) if (d0 > 0 and w0 > 0) else 0.30
        cam_h = float(t_base[2] - self.support_z)
        why_bad = []
        if cam_h <= 0.08:
            why_bad.append("相机离桌面仅 {:.0f}mm（视线太平，解不稳定）".format(cam_h * 1000))
        if info["jitter"] > diag + 0.05:
            why_bad.append("底边跨度 {:.0f}mm > 物体对角线 {:.0f}mm+50mm（吃进桌面/背景）"
                           .format(info["jitter"] * 1000, diag * 1000))
        info["ok"] = not why_bad
        if why_bad:
            info["why"] = "；".join(why_bad)
        back, why = self._backoff(cls, v_ray, tf)
        info["back"] = float(back)
        info["back_why"] = why
        info["range_front"] = r_f
        axis = p_front + v_ray * back
        axis[2] = self.support_z
        info["range"] = r_f + back
        return axis, info

    def _size_range_crosscheck(self, cls, mask, depth, k):
        """★ 已知尺寸测距（判别实验）：完全不用 TF、不用桌面、不用深度尺度。

        圆截面的**轮廓宽度恒等于直径**（与视角无关）⇒ 由掩码宽度反解轴距
            Z_size = fx · D / W_px      （D = 直径，W_px = 掩码宽像素）
        深度法在同一次测量里给出另一个轴距：
            Z_depth = p5(掩码内深度) + r  （p5 是可见面最近处，圆柱那里正好在轴外 r）
        两者都在**相机系**里、都不经 TF ⇒ 差值直接回答"深度在物体处准不准"：
          · 差几毫米 ⇒ 深度准 ✓ → 偏差在下游（相机外参平移 / 规划 / 臂）
          · 差几厘米 ⇒ 深度在物体处不准 ✗ → 偏差在感知（本轮补丁方向）
        长方体轮廓宽随偏航变，只回报"隐含宽度"与目录区间比 ✓
        """
        d, w, h = self.object_sizes.get(cls, (0.0, 0.0, 0.0))
        if depth is None or k is None or mask is None or d <= 0 or w <= 0:
            return None
        ys, xs = np.nonzero(mask.astype(bool))          # 用**原始**掩码（不腐蚀），宽度才准
        if xs.size < 20:
            return None
        # ★ 只取"物体中高那一带"：斜视角下底边比中高近几十毫米 ✗
        #   尺寸法和深度法必须量同一个点（中高处的轴距），否则差值是几何效应不是误差 ✗
        v_mid = 0.5 * (float(ys.min()) + float(ys.max()))
        band = 0.10 * max(4.0, float(ys.max() - ys.min()))
        sel = np.abs(ys - v_mid) <= band
        if int(sel.sum()) >= 20:
            xs_b, ys_b = xs[sel], ys[sel]
        else:
            xs_b, ys_b = xs, ys
        w_px = float(xs_b.max() - xs_b.min() + 1)
        fx = float(k[0])
        if fx <= 0 or w_px <= 0:
            return None
        zs = depth[ys_b, xs_b].astype(float)
        zs = zs[np.isfinite(zs) & (zs > 0.0) & (zs < self.max_range)]
        if zs.size < 20:
            return None
        # ★ 太窄时 1 px 量化误差就是几厘米（实测 42~43 px 时报出 −111/−419 mm 的假误差 ✗）
        #   ⇒ 阈值 40→70 px：低于它结论不可用，宁可跳过也不误导 ✓
        if w_px < 70:
            return dict(w_px=w_px, too_small=True)
        is_round = abs(d - w) < 0.01
        r_ax = 0.5 * (d + w) / 2.0 if is_round else 0.5 * min(d, w)
        z_dep = float(np.percentile(zs, 5)) + r_ax
        out = dict(w_px=w_px, z_dep=z_dep, implied_d=w_px * z_dep / fx,
                   expect_lo=min(d, w), expect_hi=math.hypot(d, w))
        if is_round:
            out["z_size"] = fx * (0.5 * (d + w)) / w_px      # 圆：直径/2 就是半径 r
            out["z_size"] -= 0.0
            out["diff"] = out["z_size"] - z_dep
        return out

    def _table_plane_check(self, depth, k, tf, step=40):
        """★ 桌平面校验：网格取样深度，用平面约束挑出桌面像素，反投影应恒为 support_z。

        一次同时验证：深度尺度/编码 ✓ 内参 ✓ 相机外参 ✓ 支撑面高度 ✓
        · z 随像素行线性变化（斜率大）⇒ 外参俯仰/内参有问题
        · 只是整体偏移 ⇒ 相机高度或支撑面高度有问题
        """
        if depth is None or k is None or tf is None:
            return None
        R = self._rot_of(tf)
        tr = tf.transform.translation
        t_base = np.array([tr.x, tr.y, tr.z])
        h, w = depth.shape[:2]
        rows, zs = [], []
        for v in range(step // 2, h, step):
            for u in range(step // 2, w, step):
                Zm = float(depth[v, u])
                if not np.isfinite(Zm) or Zm <= 0.0 or Zm >= self.max_range:
                    continue
                Zp, _ = self._plane_z_at(u, v, k, R, t_base)
                if Zp is None or abs(Zm - Zp) > 0.03:
                    continue                        # 不在支撑面附近 ⇒ 不是桌面像素
                fx, cx, fy, cy = k[0], k[2], k[4], k[5]
                pc = np.array([(u - cx) * Zm / fx, (v - cy) * Zm / fy, Zm])
                rows.append(float(v))
                zs.append(float((R.dot(pc) + t_base)[2]))
        if len(zs) < 20:
            return None
        zs_a = np.array(zs)
        coef = np.polyfit(np.array(rows), zs_a, 1)
        return dict(n=int(zs_a.size), mean=float(zs_a.mean()), std=float(zs_a.std()),
                    slope=float(coef[0]), cam_h=float(t_base[2] - self.support_z))


    def _contract_point(self, pts_cam, tf, cls, bottom_band=False):
        """相机系点云 → 契约点（物体中心轴 ∩ 支撑面，在 target_frame 里）。

        · 可见表面质心在相机系 → 旋转/平移进 target_frame（只用机器人内部 TF）
        · 相机与桌面等高 → 穿过物体中心的视线是水平的，**不能**用"视线∩支撑面"；
          改成：质心 + 沿【水平视线方向】后退 半个物体厚度（可见面本来就近半个厚度）
        · z 直接取支撑面高度 ✓（契约定义）
        """
        q = tf.transform.rotation
        t = tf.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # 四元数 → 旋转矩阵
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        # ★ 用【中位数】而不是平均值（2026-09-14 现场定位到的问题）：
        #   SAM2 掩码常把物体底部的桌面也吃进来，桌面像素比物体**更远** ✗
        #   → np.mean 会把物体位置整体拉远（实测：38 mm 厚的糖盒量出"横向 54 mm"，
        #     距离也被报远 5~10 cm）→ 机械臂伸手过头、手指落到物体后面 → 夹空 ✗
        #   中位数对少量/一侧的污染免疫 ✓
        p_cam = np.median(pts_cam, axis=0)
        z10, z90 = np.percentile(pts_cam[:, 2], [10, 90])
        if z90 - z10 > 0.06:
            self.get_logger().warn("  掩码深度跨度偏大（10%~90%: {:.3f}~{:.3f} m，"
                                   "跨度 {:.0f} mm）→ 多半吃进了桌面，位置已用中位数抑制 ✓"
                                   .format(z10, z90, (z90 - z10) * 1000))
        p_tgt = R.dot(p_cam) + np.array([t.x, t.y, t.z])

        # 水平视线方向（从相机原点指向物体）
        cam_o = np.array([t.x, t.y, t.z])
        v = p_tgt - cam_o
        v[2] = 0.0
        n = np.linalg.norm(v)
        if n < 1e-6:
            return None
        v = v / n

        depth = None
        if cls in self.object_sizes:
            d, w, h = self.object_sizes[cls]
            depth = min(x for x in (d, w) if x > 0)      # 可见面比轴心近"窄边/2"
        back = (depth / 2.0) if depth else 0.0

        x = p_tgt[0] + v[0] * back
        y = p_tgt[1] + v[1] * back
        return np.array([x, y, self.support_z]), p_tgt, back

    # ── 服务回调 ───────────────────────────────────────────────
    def handle_detect(self, request, response):
        response.success = False
        response.message = "not run"
        response.targets = []
        try:
            if not self._wait_streams(self.stream_timeout):
                with self._lock:
                    missing = [n for n, v in (("RGB", self._img), ("深度", self._depth),
                                              ("内参", self._info)) if v is None]
                response.message = "no image stream: " + "/".join(missing)
                self.get_logger().warn(
                    "等不到相机数据（{}）——仿真/桥接起来了吗？".format("/".join(missing)))
                return response

            if not self._ensure_pipeline():
                response.message = "vision pipeline unavailable: {}".format(self._vp_error)
                return response

            with self._lock:
                img_msg, depth_msg, info_msg = self._img, self._depth, self._info
            bgr = self._to_bgr(img_msg)
            depth = self._to_depth(depth_msg)
            if bgr is None or depth is None:
                response.message = "unsupported image encoding: {} / {}".format(
                    img_msg.encoding, depth_msg.encoding)
                return response

            tf = self._lookup_cam_to_target()
            if tf is None:
                response.message = "tf {} -> {} unavailable".format(
                    self.camera_frame, self.target_frame)
                self.get_logger().error(response.message)
                return response

            # ══════════ 检测：多帧投票（+ 抓取模式的窄词表复核）══════════
            # 帧级检测函数：返回 {class: (conf, det)}（同一类取该帧里分最高的框）
            def _frame(prompt, use_rev):
                dets = self._detect_pass(bgr, prompt=prompt, use_reviewer=use_rev)
                out = {}
                for d in dets:
                    c = (self._classify_catalog(d.get("phrase")) if prompt
                         else self._vp_mod.classify_phrase(d.get("phrase")))
                    if c is None:
                        continue
                    sc = float(d.get("score", 0.0))
                    if sc > out.get(c, (0.0, None))[0]:
                        out[c] = (sc, d)
                return out

            N = self.vote_frames
            want_pre = [c for c in (request.class_ids or []) if c in self.catalog]
            graspable = [c for c, v in self.catalog.items() if v["graspable"]]

            # ① 第 1 帧走队友默认（4 类 + 复核），用它决定要不要切抓取模式
            t0 = time.time()
            f0 = _frame(None, True)
            self.get_logger().info("第 1/{} 帧（默认 4 类+复核）: {}（{:.1f}s）".format(
                N, {k: round(v[0], 2) for k, v in f0.items()}, time.time() - t0))

            grasp_mode_on = (self.grasp_mode == "always"
                             or (self.grasp_mode == "auto"
                                 and not any(c in graspable for c in f0)))
            if grasp_mode_on:
                if 0 < len(want_pre) <= 6:
                    prompt_use, mode_txt = (" . ".join(c.replace("_", " ") for c in want_pre) + " .",
                                            "抓取模式(调用方指定 {} 类, 闭集复核)".format(len(want_pre)))
                else:
                    prompt_use, mode_txt = (self.grasp_prompt,
                                            "抓取模式(全 {} 类, 闭集复核)".format(len(self.catalog)))
                self.get_logger().warn("默认词表里没有可抓目标 → {} ".format(mode_txt)
                                       + "（召回优先；误检靠多帧投票 + 尺寸核对 + 窄词表复核压下去）")
                frames = [_frame(prompt_use, True)]           # 第 1 帧用新 prompt 重跑（复核开着）
            else:
                prompt_use, mode_txt = None, "默认(4 类 + 复核)"
                frames = [f0]

            # ② 其余帧
            for fi in range(2, N + 1):
                t0 = time.time()
                fr = _frame(prompt_use, True)
                frames.append(fr)
                self.get_logger().info("第 {}/{} 帧（{}）: {}（{:.1f}s）".format(
                    fi, N, mode_txt, {k: round(v[0], 2) for k, v in fr.items()}, time.time() - t0))

            # ③ 投票：某类要在 ≥⌈2N/3⌉ 帧出现
            need = max(1, math.ceil(N * 2.0 / 3.0))
            tally = {}
            for fr in frames:
                for c, (sc, d) in fr.items():
                    tally.setdefault(c, []).append(sc)
            # ★ 选哪一帧的框？**不能用"分数最高的那帧"** ✗（2026-09-14 现场定位到的根因）：
            #   开集标签逐帧会串（糖盒这帧叫 sugar_box、下一帧叫 potato…），
            #   而分数最高的那帧常常是"把别的物体误检成这个类"的那一帧 ✗
            #   ⇒ 最终目标位置会整体偏到别的物体上：实测同一场景两次调用，
            #     sugar_box 报 (0.335,0.001) 与 (0.378,+0.091) —— 横向差 **9 cm** ✗
            #     而手指走廊只有 ±21 mm ⇒ 必然夹空（现场"位置看着对、就是夹不起来"）
            #   改成：取该类别所有帧框心的【中位数】，再用"离中位数最近的那帧"当代表 ✓
            #   （单帧串到别的物体上就不再影响结果 ✓）并且把离散度打出来 ✓
            voted = {}
            for c, scores in tally.items():
                if len(scores) < need:
                    continue
                cds = [fr[c][1] for fr in frames if c in fr]
                cen = []
                for d in cds:
                    b = d.get("box_xyxy")
                    if b is None:
                        continue
                    cen.append((0.5 * (float(b[0]) + float(b[2])),
                                0.5 * (float(b[1]) + float(b[3]))))
                if not cen:
                    voted[c] = (float(np.median(scores)), cds[0])
                    continue
                mx = float(np.median([p_[0] for p_ in cen]))
                my = float(np.median([p_[1] for p_ in cen]))
                spread = max(math.hypot(p_[0] - mx, p_[1] - my) for p_ in cen)
                best = min(cds, key=lambda d: math.hypot(
                    0.5 * (float(d["box_xyxy"][0]) + float(d["box_xyxy"][2])) - mx,
                    0.5 * (float(d["box_xyxy"][1]) + float(d["box_xyxy"][3])) - my))
                voted[c] = (float(np.median(scores)), best)
                if spread > 25.0:
                    self.get_logger().warn(
                        "  {} 各帧框心离散 {:.0f} px（>25px）→ 已取中位数那帧，位置可能仍不稳"
                        .format(c, spread))
                else:
                    self.get_logger().info("  {} 各帧框心离散 {:.0f} px ✓（已按中位数选帧）"
                                           .format(c, spread))
            self.get_logger().info("投票（需 ≥{}/{} 帧）: {}".format(
                need, N, {c: (round(v[0], 2), "{}票".format(len(tally[c]))) for c, v in voted.items()}))
            dropped = {c: len(s) for c, s in tally.items() if c not in voted}
            if dropped:
                self.get_logger().info("  被投票淘汰（帧数不够）: {}".format(dropped))

            # ④ 抓取模式：用投票选出的候选类做一次【窄词表复核】（更准、排序也更对）
            # ★ 上限原来写 6：实测抓取模式（全 18 类开集）经常投出 7~8 个候选
            #   （同一个物体被标成好几个类）→ 复核被跳过 → 拿到的正是最差的那一批
            #   标签和位置（实测：真值在 (3.10,2.00) 的糖盒被标成 sugar_box 却报
            #   位置 (2.22,1.53)，差 1 m ✗）。放宽到 12，让复核一定跑上。
            if grasp_mode_on and not (0 < len(want_pre) <= 6) and 2 <= len(voted) <= 12:
                short = " . ".join(c.replace("_", " ") for c in voted) + " ."
                self.get_logger().info("窄词表复核（候选 {}）: {}".format(list(voted), short))
                t0 = time.time()
                # ★ 复核这一步也必须开着闭集复核 ✗（原来传 False）
                #   传 False 时它用**自由文本** phrase 重新打分 → 会把上一步闭集得到的
                #   正确标签又改回错标签：实测闭集给的是 pudding_box(0.48)，
                #   窄词表复核把整张图改判成 cracker_box(0.43) ✗
                #   复核只该"用短词表重新打分"，标签仍然由闭集 argmax 决定 ✓
                fv = _frame(short, True)
                self.get_logger().info("复核结果: {}（{:.1f}s）".format(
                    {k: round(v[0], 2) for k, v in fv.items()}, time.time() - t0))
                keep = {c: fv[c] for c in fv if c in voted}
                if keep:
                    voted = keep

            dets_all = [(v[1], bool(prompt_use), v[0]) for v in voted.values()]   # (det, own_vocab, 投票后置信度)

            want = [c for c in (request.class_ids or []) if c]
            k = info_msg.k
            # ★ 桌平面校验（一次调用一条）：深度+内参+外参+支撑面高度 四者同时验证
            if self.table_check and want:
                chk = self._table_plane_check(depth, k, tf)
                if chk is None:
                    self.get_logger().warn(
                        "  桌平面校验：找不到足够的桌面像素（相机没对着桌面？）")
                else:
                    bad = abs(chk["mean"] - self.support_z) > 0.010 or abs(chk["slope"]) > 0.002
                    self.get_logger().info(
                        "  桌平面校验: {} 个桌面像素 → 反投影 z={:.4f}±{:.4f} m "
                        "（配置 {:.3f}，差 {:+.0f} mm；随像素行斜率 {:.2f} mm/px）{}".format(
                            chk["n"], chk["mean"], chk["std"], self.support_z,
                            (chk["mean"] - self.support_z) * 1000.0, chk["slope"] * 1000.0,
                            "   ✗ 深度/外参/桌面高度有不一致，位置必然偏"
                            if bad else "   ✓ 四者一致"))

            ann_items = []          # 标注图用：(class, score, box, 轴心点)
            seen_cls = {}           # 类别 → 置信度
            for d, own_vocab, conf_voted in dets_all:
                # 抓取模式用自己的词表归类；默认路径用队友的（模块级函数）
                cls = (self._classify_catalog(d.get("phrase")) if own_vocab
                       else self._vp_mod.classify_phrase(d.get("phrase")))
                if cls is None:
                    if own_vocab:
                        self.get_logger().info("  抓取模式：phrase {!r} 对不上 objects.yaml 任何类别，丢弃"
                                               .format(d.get("phrase")))
                    continue                       # 词表外的（干扰物）直接丢
                if want and cls not in want:
                    continue
                if seen_cls.get(cls, -1.0) >= float(conf_voted):
                    continue                       # 同类已有更高置信度的框
                got = self._mask_points_cam(d.get("mask"), depth, k, cls, patch=bool(own_vocab))
                if got is None:
                    self.get_logger().warn("  {} 掩码处没有有效深度，跳过".format(cls))
                    continue
                pts_cam, ys, mstats = got
                if mstats.get("drop_far", 0.0) > 0.02:
                    self.get_logger().warn(
                        "  距离闸门[{}]: 剔除 {}% 远处像素（前缘 {:.3f} m）→ 掩码确实在吃"
                        "桌面/背景；已剔除 ✓".format(
                            cls, 100.0 * mstats["drop_far"], mstats.get("z_lo", 0.0)))
                ok_size, obs, rng = self._size_ok(cls, pts_cam)
                if not ok_size:
                    self.get_logger().warn(
                        "  {} 尺寸核对不过：实测横向 {:.3f} m 不在期望 {:.3f}~{:.3f} m → 丢弃"
                        "（多半是开集标签错了）".format(cls, obs, rng[0], rng[1]))
                    continue
                ok_h, obs_h, rng_h = self._height_ok(cls, pts_cam, tf)
                if not ok_h:
                    self.get_logger().warn(
                        "  {} 高度核对不过：实测竖直跨度 {:.3f} m 不在期望 {:.3f}~{:.3f} m → 丢弃"
                        "（标的类别高矮不符，照它抓会抓空）".format(cls, obs_h, rng_h[0], rng_h[1]))
                    continue
                seen_cls[cls] = float(conf_voted)
                # ★ 独立第二算法：**框中心像素的深度**（完全不用掩码）
                #   掩码法会被桌面/背景 bleed 拉偏，而框中心像素基本落在物体正面上 ✓
                #   两者一比就知道"位置估计是不是可信" —— 这是唯一还没验证的环节
                try:
                    box = d.get("box_xyxy")
                    if box is not None and depth is not None:
                        u0 = int(round(0.5 * (float(box[0]) + float(box[2]))))
                        v0 = int(round(0.5 * (float(box[1]) + float(box[3]))))
                        h_, w_ = depth.shape[:2]
                        r = 4
                        win = depth[max(0, v0 - r):min(h_, v0 + r + 1),
                                    max(0, u0 - r):min(w_, u0 + r + 1)]
                        win = win[np.isfinite(win) & (win > 0.0) & (win < self.max_range)]
                        if win.size >= 3:
                            zc = float(np.median(win))
                            fx, cx, fy, cy = k[0], k[2], k[4], k[5]
                            pc = np.array([(u0 - cx) * zc / fx, (v0 - cy) * zc / fy, zc])
                            q, t = tf.transform.rotation, tf.transform.translation
                            R_ = np.array([
                                [1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w), 2*(q.x*q.z + q.y*q.w)],
                                [2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z), 2*(q.y*q.z - q.x*q.w)],
                                [2*(q.x*q.z - q.y*q.w), 2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x*q.x + q.y*q.y)],
                            ])
                            pb = R_.dot(pc) + np.array([t.x, t.y, t.z])
                            pm = self._to_target_frame(pts_cam, tf).mean(axis=0)
                            self.get_logger().info(
                                "  位置核对[{}]: 掩码法 base({:+.3f},{:+.3f},{:.3f}) vs "
                                "框心法 base({:+.3f},{:+.3f},{:.3f}) → 差 ({:+.0f},{:+.0f}) mm{}"
                                .format(cls, pm[0], pm[1], pm[2], pb[0], pb[1], pb[2],
                                        (pm[0]-pb[0])*1000, (pm[1]-pb[1])*1000,
                                        "   ✗ 偏差 >30mm，掩码几何不可信"
                                        if abs(pm[0]-pb[0]) > 0.03 or abs(pm[1]-pb[1]) > 0.03
                                        else "   ✓ 两法一致"))
                except Exception as e:                      # noqa: BLE001
                    self.get_logger().warn("  位置核对失败: {}: {}".format(type(e).__name__, e))
                try:
                    xc = (None if not own_vocab else
                          self._size_range_crosscheck(cls, d.get("mask"), depth, k))
                    if xc and xc.get("too_small"):
                        self.get_logger().info(
                            "  已知尺寸测距[{}]: 掩码只有 {:.0f} px 宽（<70 px）→ 量化误差可达几厘米，结论不可用，跳过".format(
                                cls, xc["w_px"]))
                    elif xc:
                        if "z_size" in xc:
                            same = abs(xc["diff"]) <= 0.020
                            self.get_logger().info(
                                "  已知尺寸测距[{}]: 掩码宽 {:.0f} px → 轴距 尺寸法 {:.3f} m "
                                "vs 深度法 {:.3f} m → 差 {:+.0f} mm{}".format(
                                    cls, xc["w_px"], xc["z_size"], xc["z_dep"],
                                    xc["diff"] * 1000.0,
                                    "   ✓ 深度在物体处也准 ⇒ 偏差在下游(外参平移/规划/臂)"
                                    if same else
                                    "   ✗ 差 >20 mm ⇒ 深度在物体处不准，偏差在感知这一侧"))
                        else:
                            self.get_logger().info(
                                "  已知尺寸测距[{}]: 掩码宽 {:.0f} px → 隐含宽度 {:.3f} m "
                                "（目录 {:.3f}~{:.3f} m）轴距(深度法) {:.3f} m".format(
                                    cls, xc["w_px"], xc["implied_d"],
                                    xc["expect_lo"], xc["expect_hi"], xc["z_dep"]))
                except Exception as e:                      # noqa: BLE001
                    self.get_logger().warn("  已知尺寸测距失败: {}: {}".format(type(e).__name__, e))
                conv = self._contract_point(pts_cam, tf, cls)
                if conv is None:
                    continue
                point, centroid_tgt, back = conv
                # ★ 补丁③ 地面约束测距 vs 掩码法：两条独立链路的结果都打出来，
                #   差值就是"位置偏置"的自检指标（两法都偏远 ⇒ 偏置在共用的外参/桌面假设上）
                gp, ginfo = ((None, {}) if not own_vocab
                             else self._ground_point(cls, d.get("mask"), k, tf))
                if gp is not None:
                    drift = float(math.hypot(gp[0] - point[0], gp[1] - point[1]))
                    ok_p = bool(ginfo.get("ok"))
                    self.get_logger().info(
                        "  地面测距[{}]: 轴心 base({:+.3f},{:+.3f}) 前缘 {:.3f} m "
                        "后退 {:.0f}mm（{}）底边跨度 {:.0f}mm 影宽 {:.0f}mm 相机高 {:.0f}mm "
                        "→ 与掩码法差 {:.0f} mm{}".format(
                            cls, gp[0], gp[1], ginfo.get("range_front", 0.0),
                            ginfo.get("back", 0.0) * 1000.0, ginfo.get("back_why", ""),
                            ginfo.get("jitter", 0.0) * 1000.0,
                            ginfo.get("lat_half", 0.0) * 2000.0,
                            ginfo.get("cam_h", 0.0) * 1000.0, drift * 1000.0,
                            "" if ok_p else "   ✗ {}，本次不采用".format(ginfo.get("why", ""))))
                    if self.position_source == "plane" or (
                            self.position_source == "auto" and ok_p):
                        point = gp
                        centroid_tgt = ginfo["front"]
                        back = ginfo["back"]
                        self.get_logger().info("  → 采用【地面测距】({})".format(
                            ginfo.get("back_why", "")))
                    else:
                        self.get_logger().info("  → 采用【掩码法】（position_source={}）".format(
                            self.position_source))
                elif self.position_source == "plane":
                    self.get_logger().warn("  地面测距[{}]不可用（{}）→ 回退掩码法".format(
                        cls, ginfo.get("why", "?")))
                tgt = GraspTargetStamped()
                tgt.header.frame_id = self.target_frame
                tgt.header.stamp = self.get_clock().now().to_msg()   # 本次检测时刻
                tgt.class_id = cls
                tgt.confidence = float(conf_voted)      # 多帧投票后的置信度（比单帧稳）
                tgt.point.x, tgt.point.y, tgt.point.z = (
                    float(point[0]), float(point[1]), float(point[2]))
                response.targets.append(tgt)

                msg = ("  {} conf={:.3f} 可见面质心(base)=({:.3f},{:.3f},{:.3f}) "
                       "实测横向 {} / 竖直 {} 后退 {:.0f}mm → 轴心∩支撑面=({:.3f},{:.3f},{:.3f})"
                       ).format(
                    cls, tgt.confidence, centroid_tgt[0], centroid_tgt[1], centroid_tgt[2],
                    ("{:.3f} m".format(obs) if obs is not None else "n/a"),
                    ("{:.3f} m".format(obs_h) if obs_h is not None else "n/a"),
                    back * 1000, point[0], point[1], point[2])
                self.get_logger().info(msg)
                box = d.get("box_xyxy")
                if box is not None:
                    ann_items.append((cls, tgt.confidence, box, point))

            # 标注图（评分要看"框出目标 + 名称"）—— 用这一帧的原图，stamp 与目标一致
            self._publish_annotated(bgr, ann_items, img_msg.header.stamp)

            response.targets.sort(key=lambda t: -t.confidence)
            response.success = len(response.targets) > 0
            response.message = ("ok: {} target(s)".format(len(response.targets))
                                if response.success else "no detection / no class matched")
            return response
        except Exception as e:                     # 绝不把异常抛回调用方
            import traceback
            response.success = False
            response.message = "{}: {}".format(type(e).__name__, e)
            self.get_logger().error("检测失败：{}\n{}".format(e, traceback.format_exc()))
            return response


def main():
    self_test = "--self-test" in sys.argv
    rclpy.init()
    node = DetectGraspTargetNode()
    # ★ 必须多线程：服务回调里要等相机数据，而订阅回调得在别的线程继续跑，
    #   否则（单线程 executor）等不到数据就死等 ✗
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        if self_test:
            node.get_logger().info("--self-test：executor 后台转，主线程直接跑一次检测")
            th = threading.Thread(target=ex.spin, daemon=True)
            th.start()
            node._spin_for(1.0)          # 让订阅先收到至少一帧（此处不在回调里，可 spin）
            req = DetectGraspTarget.Request()
            req.class_ids = []
            resp = node.handle_detect(req, DetectGraspTarget.Response())
            print("\n=== 自测结果 success={} msg={} ===".format(resp.success, resp.message))
            for t in resp.targets:
                print("  class={:10s} conf={:.3f} frame={} point=({:.4f}, {:.4f}, {:.4f})".format(
                    t.class_id, t.confidence, t.header.frame_id,
                    t.point.x, t.point.y, t.point.z))
        else:
            ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
