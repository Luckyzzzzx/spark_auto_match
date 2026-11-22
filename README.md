# 夺宝奇兵自动赛 · 自动任务组 · 代码基线

本仓库是从 Spark-H 机器人（自动组借用调试）上拉取的 `auto_match_25` **真实代码基线**，用于版本管理和多人协作。

> 注意：机器上的代码比官方发布的 `auto_match_25.zip` 新很多，已大改（视觉识别、主逻辑、二层抓取等），但仍需先在台架上验证再运行整套任务。

## 目录结构

- `auto_match_25/auto_match/` — 主功能包
  - `nodes/` 各节点（含多个历史版本 `runtask1.py`/`runtask_7_16.py` 等，**当前只使用 `runtask.py`**）
  - `config/` `items_config.yaml`、`mark_map.pkl`、地图、`move_base_param/`
  - `launch/` `auto_match.launch`
- `auto_match_25/common/` — 动作 / 消息 / 服务
- `auto_match_25/spark_alignment_mark/` — AR 码对齐节点
- `auto_match_25/spark_move/` — 直行 / 转向控制
- `tools/` — 标定数据（`thefile.txt` / `thefile2.txt`）与桌面工具脚本
  - `tools/c_calibration/` — C 组新机器人相机取帧、蛋糕顶面四角与机械臂真实串口反馈工具；二层抓取仍需实测标定
  - `tools/c_vision/` — C 组蛋糕数据构建、模型训练审计、ROS 检测与 RGB-D 顶面几何工具

## 导航模块（B 负责）

`auto_match/nodes/nav_manager.py` —— `NavigationManager` 封装：

- **接口**：`goto(name, with_orientation=True)` / `arrive(name)` / `goto_route(drop_zone, sorting_name, phase)` / `step_go(dist)` / `step_back()`
- **定位健康**：`check_localization()` / `force_relocalize(x, y, yaw)`
- **兼容旧接口**：`goto_local(name)` / `arriveto_local(name)`，可一行替换 `runtask.py` 里的 `RobotMoveAction`
- **状态机**：`IDLE → NAVIGATING → REACHED / FAILED`
- **异常处理**：超时取消 + 卡死检测 + 自动重试（次数/阈值均可配）
- **点位**：优先读 `config/mark_map.yaml`（可读可 diff），没有则回退 `config/mark_map.pkl`；上机用 `scripts/pkl2yaml.py` 一键导出
- **路由**：`config/route_table.yaml` 把 `runtask.py` 里 Step 点的 if-else 抽成配置
- **参数**：`config/nav_params.yaml`（超时 / 重试 / 卡死 / 定位阈值），上机只改这里

替换方式（`runtask.py`）：

```python
from nav_manager import NavigationManager
self.robot = NavigationManager()   # 原来是 RobotMoveAction()
```

## 已知问题（待完善）

1. 机器人当前只有通用 `~/yolov8n.pt`，正式比赛仍需要实际场景数据训练并验证的权重。
2. 分类板 ROI 需要在分类区停车位确定后填入 `items_config.yaml`。
3. 当前机器人点位表缺少 `Collection_A`，必须在实际场地补标；完整预检会在补标前阻止自动任务启动。

## A 组（视觉感知）改动 · 2026-11-19

详见 [`auto_match_25/auto_match/docs/vision_module.md`](auto_match_25/auto_match/docs/vision_module.md)。

- `detect_ros.py` 重写：阈值/模型/推理参数改由 `config/items_config.yaml` 的 `detect` 段控制，
  同时支持 Ultralytics 与旧版 yolov5，加帧率上限、错误不再静默吞掉；`/objects` 话题契约不变。
- 新增 `nodes/board_mapping.py`（纯逻辑）+ `nodes/board_classifier.py`（ROS 节点）：
  把分类板的「图案 → B/C/D 放置区」独立成节点；到达分类区后由
  `/board_classifier/capture` 触发新一轮采集，避免读取上一局或路途中的旧结果。
- 分类板拒绝重复类别和多余目标，并支持配置 ROI；`runtask.py` 只负责触发与读取映射。
- 新增单元测试 `test/test_board_mapping.py`（不需要 ROS / 相机 / 模型）：
  `cd auto_match_25/auto_match && python3 -m unittest discover -s test -v`
- 节点执行权限已记录在 Git 中，并在 `CMakeLists.txt` 中使用 `catkin_install_python()`。

> 通用 YOLOv8 权重只用于打通运行链路。识别准确率必须用 D435 实拍数据验证，不能以“节点能启动”代替模型验收。

## 环境

- 机器人：Spark-H（借用），Ubuntu 20.04 + ROS Noetic
- 视觉环境：`~/yolo_venv`（Python 3.8 + Ultralytics）；先安装
  `auto_match/requirements-vision.txt`，启动前 source ROS 与工作区并激活该环境
- SSH：`spark@10.25.141.8`（已配置免密）

## 真机测试流程

先运行不会控制任何硬件的预检：

```bash
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
rosrun auto_match robot_preflight.py --mode vision
rosrun auto_match robot_preflight.py --mode full
```

任何 `[FAIL]` 都必须先处理。只测试 D435 和视觉链路（不启动底盘、导航或机械臂）：

```bash
roslaunch auto_match vision_test.launch start_camera:=true
rosservice call /board_classifier/capture
```

已有相机节点时使用 `start_camera:=false`，避免重复占用设备。

`mark_move.py` 会先载入已有点位，新学习是增量更新，不会清空其他点位。把机器人停稳在 A 区后补标：

```bash
rostopic pub -1 /mark_nav std_msgs/String "data: 'learn Collection_A'"
rostopic pub -1 /mark_nav std_msgs/String "data: 'finish'"
rosrun auto_match robot_preflight.py --mode full
```

只有完整预检通过、场地清空且确认只有一组 ROS/硬件进程时，才启动完整任务。任务控制明确使用 `start` / `stop`（也兼容 `1` / `0`）：

```bash
rostopic pub -1 /task_start_flag std_msgs/String "data: 'start'"
rostopic pub -1 /task_start_flag std_msgs/String "data: 'stop'"
```
