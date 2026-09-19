> **交接/接手调试先看仓库根的 [HANDOFF.md](../HANDOFF.md)** ✓（本目录是调试工具，用法见下）

# HANDOFF 工具目录（**不属于任何 ROS 包**，colcon 不会安装它）

本目录只放**离线自检工具**。这里没有 package.xml，所以放在 `src/` 下不会被 colcon 当成包 ✓。

## 现有文件

| 文件 | 用途 |
|---|---|
| `check_dining_view_geometry.py` | ★ **观察位/近看位/站位几何自检（2026-09-18 新增，问题①）**：用**真实 `map.pgm` + Nav2 `robot_radius`** 验 6 件事——四条桌沿哪一侧站得下人、近看位到桌沿的距离是否四向一致、站位解算出来的导航目标点是否落在自由区（`free(0.28)`）、幻影目标会不会劫持选目标、视野并集覆盖够不够；并带"旧实现必然失败"的回归对照。**改几何常量后先跑它，不用起仿真** |
| `test_grasp_retry.py` | ★ **抓取接触闭环离线单测（2026-09-18 新增，问题②）**：`classify_closure`（空合/夹到/无数据）、偏置阶梯的方向与顺序、真跑 `_grasp_with_offsets`（空合→换偏置→接触、全程空合必须报失败、stage=5 立刻停）、支撑面档位、尺寸核对按距离放宽。28 项断言，不用起仿真 |
| `test_table_pixel_filter.py` | ★ **桌面剔除离线回归（2026-09-18 新增）**：合成"相机高出桌面 39 mm + 掩码底边吃进物体前方桌面"的场景，跑真身 `_mask_points_cam`，断言"开/关桌面剔除"下可见面深度从 0.318 m（偏浅 132 mm ✗）恢复 0.450 m ✓；另含"整块掩码都在桌面上"与"远处物体不误剔"两个护栏用例。**不用起仿真** |
| `test_count_items.py` | ★ **基础题待计数清单离线自测（2026-09-19 新增）**：喂一个「裁判给了 cola/bowl/windex bottle」的配置，断言 `vision_pipeline.ITEM_NAMES`、`insid3_review.TARGET_CLASSES`、开集 prompt、答案 JSON 类别**同时**同步 + 别名命中(cola→coke can) + 护栏（缺参考图警告 / 配置缺失不崩 / 默认配置与原硬编码等价）。16 项断言，**不用起仿真** |
| `world_tools.py` | ★ **换世界工具（2026-09-19 新增）**：`check <world>` = 体检（缺模型 / 桌子位姿与`support_surface` 该填什么 / 桌上的物体是否都在 objects.yaml / 邻居桌值）+ 打印完整同步清单；`map <world>` = **直接从 world 栅格化出 map.pgm/yaml**（不用开车跑 SLAM ✓ 秒级，已验证与现用手工图在桌沿/过道处逐点一致 ✓）。**不用起仿真** |
| `test_grasp_phase_ctor.py` | ★ **GraspPhase 构造冒烟（2026-09-19 新增，事故驱动）**：用假 node 真构造一遍 `GraspPhase` —— 拦住「新加的一行写在它依赖的成员之前 ⇒ 构造抛异常 ⇒ 整个 Phase 2 不跑」这类 bug（`py_compile` 与其它单测都拦不住 ✗）。20 项断言，**不用起仿真** |
| `test_shape_relabel.py` | ★ **标签按实测尺寸纠正（2026-09-19 新增）**：闭集分类器把 84 mm 的扁罐头叫成 `chips_can`(0.25 m)/`sugar_box`(0.175 m)，而 `_height_ok` 只会否决不会纠正 ⇒ 调用方永远拿不到它。`relabel_decision` 只在标签**明显不自洽**（偏差 > max(30 mm, 45%·标签高度)）**且**调用方要的类贴合（≤ max(12 mm, 18%)）**且**无更接近的类时才改标；另含驱动侧 `wrong_object` 判据（实测间距比目标窄边宽 >4 mm ⇒ 抓错物体，不算成功）。16 项断言，**不用起仿真** |
| `test_grasp_mode_switch.py` | ★ **视觉模式判据离线自测（2026-09-19 新增）**：`needs_grasp_mode` —— "调用方点了类就必须按它判"（旧判据只看默认 4 类词表里有没有可夹的东西 ⇒ 现场 8 次调用全 `no class matched`）。含现场 8 帧的回归用例（新判据 8/8 会切、旧判据 0/8）。13 项断言，**不用起仿真** |
| `test_table_yaw_probe.py` | ★ **桌腿基准离线自测（2026-09-19 新增）**：`table_legs_map`（世界文件真值逐条核对）、`legs_from_scan`（半径匹配 + 方位窗：belief 偏航偏 25.7° 也能 4/4 命中）、`fit_pose_from_landmarks`（2D Kabsch 反解真偏航，误差 0.07°）、以及 `_apply_table_yaw_fix` 的**注入值符号**（e = belief − truth，反了会把朝向搞坏 ✗）。33 项断言，**不用起仿真** |
| `test_closure_phase.py` | 合拢开始/结束判定与阶段归属（`analyze_closure` / `format_closure_report`）+ 假对象真跑 `call_grasp` + ★**用例⑩ `closing_span_mm`（合拢轴偏 φ 时长方体需要的开口；φ=19.8° = 80 mm 开口上限）**（41 项断言）|
| `test_position_math.py` | 位置解算离线自测（合成 z-buffer 场景跑真身函数：柱/盒轴心误差、地面约束测距拒绝条件、桌平面校验）|
| `check_grasp_geometry.py` | **顶抓几何自检**：用 URDF 的真实碰撞几何（fr3_hand 网格 + 两根手指的 8 个碰撞盒）把整只手摆到抓取位姿，逐个采样角做精确碰撞检测 → 回答"这个物体的 `grasp_lift` 能不能规划成功" |
| `vision_probe.py` / `capture_grasp.py` / `sweep_target.py` / `sweep_offset.py` | 视觉探针 / 抓取取证（带相机存图）/ 抓取点扫描 / 径向偏置扫描（**要起仿真**）|
| `_map_table3_zoom.png` / `_map_waypoints.png` | 地图取证图（餐桌脚印 + 巡逻点） |
| `patches/` | 迁移期的临时补丁留档（`multiframe_fusion_2026-09-18.patch` 含两个真 bug 修复，见交接文档 §7）|

### 30 秒离线验"仿真里到底有没有 mimic"（2026-09-18）

```bash
cd ~/Robocup@home_ws && source install/setup.bash
python3 -c "import xacro; open('/tmp/r.urdf','w').write(xacro.process_file( \
  'src/turtlebot3_manipulation_gazebo/urdf/turtlebot3_manipulation.urdf.xacro', \
  mappings={'use_sim':'true'}).toxml())"
ign sdf -p /tmp/r.urdf > /tmp/r.sdf && grep -c mimic /tmp/r.sdf
#  输出 0 ⇒ mimic 没进 SDF（Fortress/sdformat12 不支持）⇒ 第二指必须自己受控，
#            见 docs/handoff/HANDOFF_nav_survey_and_grasp_tolerance_2026-09-18.md §10
```

### 不用起仿真的三条命令（改完代码先跑）

```bash
cd ~/Robocup@home_ws && source install/setup.bash
python3 src/HANDOFF_harness/check_dining_view_geometry.py   # 几何/目标选择（全绿 ✓）
python3 src/HANDOFF_harness/test_grasp_retry.py             # 接触判据/偏置阶梯（28 项 ✓）
python3 src/HANDOFF_harness/test_closure_phase.py           # 合拢判定 + closing_span_mm（47 项 ✓）
python3 src/HANDOFF_harness/test_table_yaw_probe.py         # 桌腿基准 + 偏航补偿注入（35 项 ✓）
python3 src/HANDOFF_harness/test_grasp_mode_switch.py       # 视觉模式判据（13 项 ✓）
python3 src/HANDOFF_harness/test_grasp_phase_ctor.py        # 构造冒烟（20 项 ✓）
python3 src/HANDOFF_harness/test_shape_relabel.py           # 改标/抓错物体/抖动半径（22 项 ✓）
python3 src/HANDOFF_harness/test_count_items.py             # 待计数清单三处联动（16 项 ✓）
```

## ★ 已删除：`sim_harness.sh` 与 `grasp_client.py`（2026-09-14）

那套"无头取证台"是旧工作区带过来的：它自己 spawn 机器人、用 `ign model -m <物体> -p`
读 **gz 真值** 来判"物体有没有被抬起来"，还用一条写死的 static TF 伪造 `map→base_footprint`。

**用户明确要求：不许用 gz 真值，调试也不行** —— 它会让"视觉/规划到底行不行"这个结论失效
（真值 = 直接读仿真答案 ✗）。所以整条删除，并同步删掉了抓取链里的真值退路
（`grasp_phase.truth_targets` / `--truth-only` / `--no-truth-fallback` / 适配层的 `debug_truth`）。

它原本的用途现在有**更真实**的替代，都在真仿真 + 真导航栈里跑，不读任何真值：

```bash
# 真·端到端（Gazebo + Nav2/AMCL + 视觉 + 抓取服务）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
source ~/venvs/vision_env/bin/activate
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
ros2 run turtlebot3_manipulation_navigation2 patrol_task.py          # 比赛主流程（Phase 1 + 2）

# 只验抓取阶段（等价于旧取证台"机器人已停好、只测服务链路"的用法）
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py --skip-nav --no-creep
```

判"到底抓没抓起来"也别再读真值：**看 Gazebo GUI / 相机图像**，或看
`/joint_states` 里手指关节的到位情况（`fr3_finger_joint1` 卡在物体半宽处 = 夹住了、
一路合到目标值 = 空合）。旧 README 里"罐子 z 抬升 ≥ 3 cm"那条判据是读真值得来的，
**已作废**（但它记录的失败现象仍然有效：6 次实跑里只有 2 次真抓起、且都被带倒 →
根因就是本文档下面那条抓取高度问题）。

## 规划失败时先跑这个

`grasp pose IK (0/25): eef in collision: fr3_hand - object` 这类报错**与视觉精度、定位、
真值完全无关**：MTC 的 `ComputeIK` 在算 IK **之前**就把手按目标位姿摆好做碰撞检查
（`isTargetPoseCollidingInEEF`，见 `moveit_task_constructor/core/src/stages/compute_ik.cpp`），
手和物体一相交就 25 个角全拒。典型根因是 `objects.yaml` 的 `grasp_lift` 让物体顶段插进了手掌
（手掌只到指尖平面上方 37.4 mm）。改 `grasp_lift` 前后各跑一次体检：

```bash
source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
python3 src/HANDOFF_harness/check_grasp_geometry.py            # 全表体检
python3 src/HANDOFF_harness/check_grasp_geometry.py --sweep mustard_bottle
```
