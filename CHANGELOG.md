# 变更日志

## 2026-11-23 · A组（视觉）· 按机器人实测环境修正

### 背景（`10.24.170.0` 巡检结论）
- `~/auto.pt`、`~/banana.pt` 是 **YOLOv5** 权重，`ultralytics 8.x` 拒绝加载（"NOT forwards compatible"）；
- `~/yolov8n.pt` 不存在 → 原配置 `backend: ultralytics + model_auto: ~/yolov8n.pt` 等于没有模型；
- `~/yolo_venv` 不存在 → `run_detect_ros.sh` 直接 `exit 2`，视觉链路整体起不来；
- 系统 `python3` 3.8.10 已装 `ultralytics 8.0.143` + `yolov5 7.0.13` + `torch 2.0.1`（CPU），不需要 venv；
- 蛋糕权重 `cake_dataset_v4_20261122/.../best.pt` 是 YOLOv8、单类 `cake`，可加载但未接入主链路。

### 修复
- `config/items_config.yaml`：`detect.backend` 改 `yolov5`，`model_auto: ~/auto.pt`、`model_banana: ~/banana.pt`。
- `scripts/run_detect_ros.sh`：解释器改成「显式变量 → `~/yolo_venv` → 系统 python3」回退链，
  并加后端自检（`ultralytics`/`yolov5` 都没有则 `exit 3`），不再因为缺 venv 直接退出。
- `docs/vision_module.md`：新增「机器人实测环境」与「部署到机器人」两节。

### 部署
- 已按上表把视觉相关文件同步到 `~/spark_noetic/src/3rd_app/auto_match_25/auto_match`（先整包备份），
  未改动 `runtask.py` / `nav_manager.py`（依赖 B 组点位）。

### 追加（同日）· 一套配置通吃两台机器人
- `detect.models` 支持**模型清单**：按顺序尝试、文件不存在自动跳过；
  每个模型可单独指定 `backend`（默认 `auto`：先 ultralytics 再 yolov5）和 `aliases`。
- 起因：`10.25.141.8` 上没有 `auto.pt`/`banana.pt`，只有通用 `~/yolov8n.pt`（COCO）
  与 `~/cake_best.pt`/`~/cake_v4_best.pt`；而 `10.24.170.0` 只有自训练 YOLOv5 权重。
  单模型写法必然有一台跑不了。
- 蛋糕模型接入 `/objects`（id=5）：两台机器的 cake 权重路径都列进清单，
  实机自检从「缺少 `['cake']`」变为「自检通过: wine/cake/banana/bear/clock」。
- 实机（170.0）加载结果：`models=[yolov5(7类), ultralytics(41类), ultralytics(1类)]`。

### 追加（同日）· 两台机器人的分工与 1 号机补齐
- 约定：**1 号机 = 10.24.170.0（主用）**，**2 号机 = 10.25.141.8（副机）**。
- 1 号机原本缺 B/D 组整条链（`nav_manager`/`nav_route`/导航配置/preflight/测试/文档），
  已从仓库统一部署补齐（不是从 2 号机互拷，避免把 2 号的点位带过去）。
- **未覆盖** 1 号机自己的 `config/mark_map.pkl`（14 个点，含 `Collection_A`）与 `test_map.*`。
- 1 号机 14 个历史遗留脚本（`runtask818.py`、`runtask (copy).py` 等）已移到
  `~/legacy_auto_match_nodes_20261123/`。
- `scripts/robot_preflight.py` 改为环境无关：支持 `detect.models` 清单、
  按「环境变量 → `~/yolo_venv` → 系统 python3」选择解释器，缺模型只告警不报错。
- 实机结果（1 号机）：`--mode vision` 与 `--mode full` **均通过**，
  可用模型 3 个，点位 14 个。

### 追加（同日）· 分类板抗误判 + ROI 标注工具
- `board.min_size_ratio`（默认 0.55）：三格检测框的**尺寸一致性**检查。
  分类板三个图案等大共面，中央 3×3 金砖远近不同 → 框大小差异明显。
  这条不依赖 ROI 标定，能挡住「把金砖当成分类板」，状态名 `size`。
- `normalize_detections` 改为保留框宽高（5 元组），拿不到尺寸时自动跳过该检查（向后兼容 2/3 元组）。
- 新增 `scripts/measure_board_roi.py`：在分类区停车位采 8 秒，只统计通过全部护栏的帧，
  取 10%~90% 分位外接矩形 + 余量，打印或 `--write` 写回 `items_config.yaml`（自动 `.bak`）。
- 单测 26 → **40 个**（新增尺寸一致性 6 个）；实机（1 号机）单测 40 passed、预检通过。

## 2026-11-19 · 自动组（B 导航 / D 调度）

### 修复
- **抓取导航点写错（致命）**：`runtask.py` 循环里 `goto_local("Collection_A")` → `goto_local(sorting_name)`。原逻辑每次抓取都跑向 A 收取区，抓不到金砖。
- **local_costmap 障碍/膨胀层未生效**：`local_costmap_params.yaml` 的 `plugins` 缩进到顶层，改为 `local_costmap` 命名空间下。
- **代价地图参数**：`costmap_common_params.yaml` 障碍层启用、`inflation_radius 0.01→0.30`、激光高度 `2.5/4.0→0/2.0`。

### 变更
- **导航模块接入**：`runtask.py` 由 `RobotMoveAction` 切换到 `NavigationManager`（直接调 move_base action，带超时/卡死检测/重试/状态机）。旧类保留可回滚。
- **新增导航文件**：`nodes/nav_manager.py`、`config/nav_params.yaml`、`config/mark_map.yaml`、`config/route_table.yaml`、`scripts/pkl2yaml.py`。
- **防死循环**：分拣点耗尽时 break，避免 IndexError。
- **堆叠补全**：`drop2()` 后递增 `drop_times`，支持 3 层以上。
- **健壮性**：标定文件读取容错、`wait_for_service` 加超时、废弃函数标注。

### 验证（实机，无场地）
- 相机 D435 30Hz、雷达 YDLIDAR G6 10Hz、底盘 odom、机械臂 swiftpro 动作/气泵 —— 全部正常
- `catkin_make` 0 错误
- 导航栈 `map_server` / `amcl` / `move_base` 全部在线

### 遗留（A 组）
- `items_config.yaml` 缺 `cake` 类别，`runtask.py` 的 `CAKE_ID=5` 永不命中 → "蛋糕先放 A"规则无法完成
- 缺 `yolov5` 包、`auto.pt` / `banana.pt` 模型 → `detect_ros` 启动失败

### 待验证
- 以上改动仅语法/逻辑检查，实机行为需等机器人空闲 + 场地验证（导航接入、3 层堆叠、蛋糕优先 A）。
