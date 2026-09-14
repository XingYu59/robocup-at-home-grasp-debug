# GraspTargetStamped.msg
#
# 「视觉 → 抓取」的目标契约。
# 提供方：视觉侧节点（复用 vision_pipeline.py 的检测 + patrol_task.py 的
#         深度反投影 / TF 逻辑），必须跑在 ~/vision_env 里（torch 依赖）；
#         对外通过服务 DetectGraspTarget.srv 返回本消息。
# 使用方：编排节点（透传）与抓取侧 grasp_node。
# 解耦点：抓取侧完全不关心像素、掩码、深度图、相机内参。
#
# ══════════ 语义约定（两边必须一致，不要各自解释）══════════
#
# 1) header.frame_id = point 所在帧。
#
#    ★ 必须用【相对底盘】的帧，推荐二选一：
#        "camera_rgb_optical_frame"  ← 首选：视觉在相机系直接给点
#        "base_footprint"            ← 也可以（视觉侧若已换算好）
#      **不要用 "map"**。
#
#    为什么：抓取真正需要的是「物体在 base_footprint 下的位姿」，这是**相对量**；
#    底盘在 map 里的绝对位置不该进入抓取的精度。而 map 那一段 TF
#    （map→odom→base_footprint）含有 AMCL 的定位误差，用 map 系目标
#    等于把定位误差直接搬进抓取。用相机系/base 系时，变换链是
#    「相机→base_link→base_footprint」这种**机器人内部**的固定变换，
#    完全不含 AMCL/里程计，底盘停在哪都不影响精度（只影响够不够得着）。
#
#    抓取侧仍然负责 TF：<frame_id> → base_footprint（= MoveIt 规划帧），
#    并用 header.stamp 查询。
#
# 2) point = 物体【中心轴与支撑面的交点】。
#    对"立在桌面上的物体"，它就是碰撞箱中心的【正下方】那一点：z = 支撑面高度。
#    抓取侧据此算箱心：box_center = point + (0, 0, box_height / 2)，
#    其中 box_height 由 class_id 查 objects.yaml。
#
#    ⚠ 不要把"可见表面上的点"填进来。掩码质心 + 掩码处深度中值给的是相机可见
#      表面上的一个点（对罐子约等于近侧面），拿它当箱心会让箱子沿视线方向偏
#      约半个物体厚度（罐子半径 0.033 → 偏 3.3cm），MoveIt 就会让手指在离物体
#      3cm 处闭合。求法：由 camera_info + 掩码质心像素得到视线，与该支撑面
#      （z = 支撑面高度）求交，交点即本 point。
#
# 3) class_id = 类别名，取值就是视觉侧 vision_pipeline.ITEM_NAMES 的【原样字符串】：
#        "apple"  /  "coke can"  /  "bowl"
#    （classify_phrase() 的输出即这三者之一，含空格。）
#    本工程约定【不改视觉侧的词表】，所以：
#      * 抓取侧 objects.yaml 的 key 必须原样照抄这三个字符串
#        （YAML 里带空格要加引号："coke can": {...}）；
#      * 视觉侧以后在 ITEM_NAMES 里加物品，objects.yaml 同步加同名 key 即可。
#
# 4) stamp = 本次检测的时刻（不是发布时刻）。取目标走的是服务、且是停稳后按需调用，
#    所以正常情况一定是新鲜的；抓取侧仍会做一次新鲜度校验并拒收过旧数据。
#
# 5) 物体的尺寸/抓取参数不在本消息里 —— 由 class_id 查表得到（抓取侧负责）。
#    朝向也不在这里：立在桌面上的物体绕 z 的朝向对抓取无意义，抓取侧自己采样。

std_msgs/Header header        # frame_id = point 所在帧（"map"）；stamp = 本次检测时刻

string  class_id              # 取值 = vision_pipeline.ITEM_NAMES 原样（含空格），见上
float64 confidence            # 检测置信度 [0, 1]

geometry_msgs/Point point     # 物体中心轴 ∩ 支撑面（z = 支撑面高度），语义见上
