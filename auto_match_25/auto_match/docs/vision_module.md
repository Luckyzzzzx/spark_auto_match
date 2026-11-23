# 视觉感知模块说明（A 组）

> 对应分工表第一行：`detect_ros.py`、`items_config.yaml`、分类板识别（新增）。
> 依据：《赛项规则拟定版》自动任务赛 + 《夺宝奇兵自动赛-说明文档》
> 最后更新：2026-11-23（按机器人实测环境修正后端与启动脚本）

## 0. 机器人实测环境（2026-11-23 巡检 `10.24.170.0`）

| 项 | 实测值 | 对代码的影响 |
| --- | --- | --- |
| Python | `/usr/bin/python3` **3.8.10** | 不需要 venv |
| 后端包 | `ultralytics 8.0.143`、`yolov5 7.0.13`、`torch 2.0.1+cu117`（**CPU**，无 GPU） | 两个后端都能跑 |
| `~/yolo_venv` | **不存在** | `run_detect_ros.sh` 旧版会直接 `exit 2` → 已改为回退系统 python3 |
| 权重 | `~/auto.pt`(14M)、`~/banana.pt`(6.4M) 都是 **YOLOv5** 格式 | `backend: ultralytics` 会报 “NOT forwards compatible” → 改用 `backend: yolov5` |
| `~/yolov8n.pt` | **不存在** | 原配置指着它，等于没有模型 |
| 蛋糕权重 | `~/cake_dataset_v4_20261122/runs/cake_v4/weights/best.pt`（YOLOv8，`{0: 'cake'}`） | 可用，但目前只发布到 `/cake_detections`，主链路没订阅 |
| 现役部署 | `~/spark_noetic/src/3rd_app/auto_match_25/auto_match`（旧版：yolov5 后端、无 `board_classifier.py`） | 仓库新链路尚未部署 |

结论：**当前唯一能跑的组合是 `yolov5` 后端 + `auto.pt`/`banana.pt`**，配置已按此调整。

## 0.1 两台机器人权重不同 → 改用「模型清单」

`10.25.141.8` 上没有 `auto.pt`/`banana.pt`，只有通用 `~/yolov8n.pt`（COCO）+ `~/cake_best.pt`、
`~/cake_v4_best.pt`。所以配置里不再写死单个 `model_auto`，改成清单：

```yaml
detect:
  models:
    - path: ~/auto.pt                     # 170.0 自训练 YOLOv5
    - path: ~/banana.pt                   # 170.0 香蕉专用
    - path: ~/cake_v4_best.pt             # 141.8 蛋糕
    - path: ~/cake_dataset_v4_20261122/runs/cake_v4/weights/best.pt   # 170.0 蛋糕
    - path: ~/cake_best.pt                # 141.8 旧蛋糕（兜底）
    - path: ~/yolov8n.pt                  # 141.8 通用 COCO（兜底）
      aliases: {"teddy bear": bear, "wine glass": wine, "bear": wild_bear}
```

规则：

* **文件不存在自动跳过**（记 info，不刷 error）；
* `backend` 省略 = `auto`：先试 ultralytics，失败再试 yolov5（v5 权重 ultralytics 会明确拒绝）；
* `aliases` **只对 COCO 权重生效**，自训练权重的类别名已经是项目内部名，别名表留空即可
  —— 2026-11-23 踩过一次：全局 `"bear": wild_bear` 把玩偶改名，启动自检直接报缺 `bear`。

实机（170.0）加载结果：

```
[INFO] 加载模型(yolov5): /home/spark/auto.pt
[INFO] 加载模型(ultralytics): /home/spark/banana.pt
[INFO] 加载模型(ultralytics): /home/spark/cake_dataset_v4_20261122/.../best.pt
[INFO] 模型类别自检通过: ['wine', 'cake', 'banana', 'bear', 'clock']
[INFO] detect_ros 就绪: models=[yolov5(7类), ultralytics(41类), ultralytics(1类)]
```

## 0.2 两台机器人（2026-11-23 约定）

| | **1 号机** | **2 号机** |
| --- | --- | --- |
| IP | `10.24.170.0` | `10.25.141.8` |
| 角色 | 主用/比赛 | 副机/训练与调参 |
| 系统 python3 | ultralytics 8.0.143 + yolov5 7.0.13 + torch 2.0.1（CPU） | 只有 ROS 基础包，**没有**推理库 |
| `~/yolo_venv` | 无（回退系统 python3） | 有（1.6G，ultralytics 8.4.215 + yolov5 7.0.13） |
| 物品模型 | `~/auto.pt`、`~/banana.pt`（自训练 YOLOv5） | 无，只有 `~/yolov8n.pt`（COCO） |
| 蛋糕模型 | `~/cake_dataset_v4_20261122/runs/cake_v4/weights/best.pt` | `~/cake_v4_best.pt`、`~/cake_best.pt` |
| 部署路径 | `~/spark_noetic/src/3rd_app/auto_match_25/auto_match` | 同（各自独立） |
| 点位文件 | 自己的 `config/mark_map.pkl`（14 个点，含 `Collection_A`） | 自己的 `config/mark_map.yaml`（14 个点） |

**迁移注意（2026-11-23 已执行）**：

* 代码统一从仓库部署（不要在两台机器间互拷 `nodes/`，会把对方的点位/参数带过去）；
* **不要覆盖** `config/mark_map.*`、`config/test_map.*` —— 每台机器的地图与点位是各自标定的；
* 1 号机已补齐 2 号机原有的 B/D 组基础设施：`nav_manager.py`、`nav_route.py`、
  `nav_params.yaml`、`route_table.yaml`、`robot_preflight.py`、`route_report.py`、`pkl2yaml.py`、
  `test/`、`docs/`，以及 14 个历史遗留脚本已移到 `~/legacy_auto_match_nodes_20261123/`。

## 0.3 分类板 ROI：为什么必须标、怎么标

**风险**：分类板和中央金砖区用的是同样三种图案（玩偶/时钟/香蕉），共用同一个相机。
机器人在分类区停车时，如果画面里同时出现板子和后面的金砖，`board_classifier` 光靠类别分不出来。

**已有多重护栏**（任何一条不满足就不投票）：

| 护栏 | 配置 | 挡住什么 |
| --- | --- | --- |
| 数量必须正好 3 | `board.expect_num` | 画面里金砖+板子同时可见（4 个以上）→ `extra` |
| 三格间距均匀 | `board.min_gap_ratio` | 斜视透视压缩、凑数 → `geometry` |
| **三格尺寸一致**（新增） | `board.min_size_ratio` | **把远近不同的金砖当成板子** → `size` |
| 不重复类别 | 内置 | 同一图案被认成两次 → `duplicate` |
| 只看指定类别 | `board.include_names` | 蛋糕/场地标记 → 直接丢弃 |
| 连续多帧一致 | `board.stable_frames` | 单帧抖动 |

`min_size_ratio` 是**不依赖标定**的那一条：板上三个图案等大且共面，框的等效边长应该接近；
中央 3×3 金砖离相机远近不同，框大小差异明显。设 `0` 可关闭。

**标定 ROI（最后一道防线）**：

```bash
# 机器人停在 Classification_area 后
rosrun auto_match measure_board_roi.py --seconds 8          # 只测量并打印建议值
rosrun auto_match measure_board_roi.py --seconds 8 --write  # 满意后写回 items_config.yaml（自动 .bak）
```

脚本会连续采帧，只统计**通过上面所有护栏**的帧，取 10%~90% 分位的外接矩形再留 `--margin` 余量，
避免单帧离群把 ROI 拉大。输出示例：

```
== 采集统计 ==
  总帧数 61，通过校验 47（77%）
  丢弃 incomplete  14 帧

== 建议 ROI（像素，x1, y1, x2, y2）==
  roi: [412, 305, 968, 521]
  边长 556 x 216 px，四周余量 25 px
```

写回后再验证一次：

```bash
rosservice call /board_classifier/capture
rostopic echo /board_mapping -n1     # 出 mapping 且日志无 extra/geometry/size 即通过
```

> ROI 绑定「那个停车位 + 那个相机姿态」，停偏 10cm 就可能框不住；所以余量不要省，宁可框大一点。

## 1. 文件与职责

| 文件 | 状态 | 说明 |
| --- | --- | --- |
| `nodes/detect_ros.py` | 重写 | 支持 Ultralytics/旧 yolov5；阈值/模型/推理参数从 yaml 读；帧率上限；重复框合并；类别自检；`/objects` 契约不变 |
| `config/items_config.yaml` | 扩充 | `items` / `objects` / `detect` / `board` / `board_fallback_zone` |
| `nodes/board_mapping.py` | 新增 | 分类板映射的纯逻辑（不依赖 ROS，可单测） |
| `nodes/board_classifier.py` | 新增 | `/objects` → `/board_mapping`（JSON，latched） |
| `test/test_board_mapping.py` | 新增 | 26 个单元测试，不需要 ROS / 相机 / 模型 |
| `docs/vision_module.md` | 新增 | 本文件 |

## 2. 规则对齐（这一轮改动的依据）

规则里和视觉直接相关的条款，以及代码里对应的位置：

| 规则 | 代码对应 | 说明 |
| --- | --- | --- |
| 一层 3×3 共 9 块：玩偶×3、时钟×3、香蕉×3，间隔 4.5cm | `detect_ros.py::_merge_duplicates` | 两个模型各检一次会有重复框，重复框会让下游反复挑同一个位置，按 IoU>0.5 合并保高分 |
| 二层中间 1 块**蛋糕** | `items.cake = 5` | 新增类别；启动时自检模型到底认不认 |
| 蛋糕**必须先放 A 区**，否则本局不计分 | `board.fixed_by_name: {cake: Collection_A}` | 映射里直接注入 `cake -> Collection_A`，`runtask.py` 的 `self.mapping.get(item_type, ...)` 无需改动就能拿到 A |
| 分类板 0.34×0.1m，只有**玩偶/时钟/香蕉**三种图案，顺序每局随机 | `board.include_names: [bear, clock, banana]` | 白名单，蛋糕/场地标记/误检都进不了映射 |
| 板上三格等宽，从左到右 = B/C/D | `board.zones` + `board.min_gap_ratio` | 板子小、易斜视；三格间距明显不均匀就判 `geometry`，不投票 |
| 分类板每局更换 | `/board_classifier/capture` + `capture_id` | 到达分类区后触发新采集，拒绝上一局 latched 结果 |
| 10 分钟要抓 10 块，单块留给识别的时间很短 | `detect.max_rate_hz`、`augment`、`img_size` + 5 秒一次的 fps 日志 | 现场按日志里的 fps 决定要不要关 augment |

## 3. 数据流

```
/camera/color/image_raw
        │
        ▼
   detect_ros.py  ──► /objects (vision_msgs/Detection2DArray，按置信度降序)
        │                 │
        ▼                 ▼
   /debug_image     board_classifier.py ──► /board_mapping (std_msgs/String, latched)
                                ▲                    │
                                └── /board_classifier/capture（到达分类区后触发）
                                                │
                                                ▼
                                        runtask.py（D 组）
```

## 4. 接口契约

### 4.1 `/objects`（字段未改，仅顺序变化）

| 字段 | 含义 |
| --- | --- |
| `detections[i].results[0].id` | 物品 ID（`items_config.yaml` 的 `items` 段） |
| `detections[i].results[0].score` | 置信度 |
| `detections[i].bbox.center.x / y` | 框中心（像素） |
| `detections[i].bbox.size_x / size_y` | 框宽高（像素） |

两点变化：

* **顺序按置信度从高到低**（以前是模型输出顺序）。语义相同，下游不要依赖顺序。
* 同一位置若被两个模型各检一次，只发一个（保高分）。

### 4.2 `/board_mapping`（新增）

`std_msgs/String`，JSON，`latch=True`：

```json
{
  "mapping": {"5": "Collection_A", "3": "Collection_B", "1": "Collection_C", "4": "Collection_D"},
  "zones": ["Collection_B", "Collection_C", "Collection_D"],
  "stable": true,
  "capture_id": 1,
  "items": [[3, 120.0], [1, 240.0], [4, 360.0]],
  "stamp": 1763456789.12
}
```

* `mapping`：物品 ID（字符串）→ 放置区。**包含蛋糕（固定 A）**，所以 D 直接 `mapping.get(item_type, default)` 即可；
* `items`：本帧参与映射的 `[id, x]`，调试用；
* 只有连续 `board.stable_frames` 帧结果一致才发布；数量不等于三、类别重复或间距不均匀都不发布；
* `capture_id` 用于区分本次采集和上一局 latched 结果。

## 5. 配置速查

| 键 | 默认 | 什么时候改 |
| --- | --- | --- |
| `detect.conf_thres` | `0.5` | 误检多 → 调高；漏检多 → 调低 |
| `detect.class_conf` | `{}` | 单类覆盖，如 `{cake: 0.45}` |
| `detect.backend` | `ultralytics` | 机器人现有 `yolo_venv`；恢复旧环境时可改 `yolov5` |
| `detect.augment` | `false` | CPU 实时运行默认关闭；精度不足再对比开启 |
| `detect.img_size` | `640` | 降到 `416/320` 也提速，小图案会变差 |
| `detect.max_rate_hz` | `8.0` | 主动丢帧，防止 `/objects` 越积越旧 |
| `detect.check_classes` | `[]` | 留空 = 检查 `items` 里所有非 `*_site` / `base` 的类别 |
| `board.include_names` | `[bear, clock, banana]` | 板上图案种类变了才改 |
| `board.fixed_by_name` | `{cake: Collection_A}` | 规则改了蛋糕归属才改 |
| `board.min_gap_ratio` | `0.45` | 老是判 `geometry` → 降到 `0.35` 或设 `0` 关掉 |
| `board.stable_frames` | `3` | 现场抖动大 → 5 |
| `board.x_tolerance` | `30` | 板子离得近、图案挤在一起 → 调小 |
| `board.roi` | `[]` | 分类区停车位确定后填 `[x1,y1,x2,y2]`，排除板外目标 |
| `board_fallback_zone` | `Collection_B` | 识别失败时 B/C/D 的兜底 |

## 6. 上机步骤与验证

```bash
cd ~/spark_noetic/src/3rd_app/auto_match_25/auto_match
source ~/yolo_venv/bin/activate
pip install -r requirements-vision.txt
cd ~/spark_noetic && catkin_make
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash

# a. 单测（不用 ROS）
cd src/auto_match && python3 -m unittest discover -s test -v      # 24 passed

# b. 只看识别：启动日志里先看「模型类别自检」
rosrun auto_match detect_ros.py
#   期待看到：模型类别自检通过: ['bear', 'wine', 'clock', 'banana', 'cake']
#   若报「模型缺少这些类别 ... ['cake']」，说明权重里没有蛋糕，二层那块抓不到
rostopic hz /objects
rostopic echo /objects -n1

# c. 对着分类板看映射
rosrun auto_match board_classifier.py
rosservice call /board_classifier/capture
rostopic echo /board_mapping
#   期望：{"mapping": {"5": "Collection_A", ...}, "items": [[...]], "stable": true}
#   只有两种图案时不会发布，日志会打「本帧 N 个，需要 3 个」

# d. 整套
roslaunch auto_match auto_match.launch
```

判读要点：

* 日志 `检测帧率 x.x fps`：低于 1.5 fps 就建议把 `augment` 关掉；
* `白名单外的检测已忽略: ids=[5]`：蛋糕被检到了，正常（它本来就不该上板）；
* `分类板三格间距不均匀(0.3x)`：机器人站位偏了，或者少检了一个图案。

## 7. 已知风险 / 前提

1. **正式比赛权重不在仓库。** 当前机器人只有通用 `~/yolov8n.pt`，可用于打通运行链路，但仍需用 D435 实拍数据训练或验证正式权重。
2. **`cake` 这一类的权重是否存在？** 规则里第四类物品是蛋糕，但旧代码/旧模型是 `wine`。
   现在 `items` 里两者都保留，`objects_d` 指向 `cake`。如果模型没有 cake：
   * 启动日志会明确告警；
   * 需要重训/换权重，或把 `objects_d` 改回 `wine`（一行）先保证能抓三样。
3. `vision_msgs` 的 `ObjectHypothesisWithPose.id` 在 Noetic 是整型；若装的是字符串型，`detect_ros.py` 里改一行 `str(item_id)`。
4. 每次到达分类区必须调用 `/board_classifier/capture`；`runtask.py` 已通过 `capture_id` 拒绝上一局的 latched 结果。

## 8. 蛋糕优先（已在 `runtask.py` 改，可一键回退）

规则原文：「赛队进行金砖抓取时，需**首先**将蛋糕金砖放入得分区 A 区，否则蛋糕金砖本局比赛不计入得分。」

原来 `runtask.py` 是第 9 次（`total_drop_times == 8`）才去二层、才送 A，而且判断用的是 `BANANA_ID`
——那是旧规则（香蕉固定 A）。现在改成：

```python
self.CAKE_ID = 5        # items_config.yaml 里 cake: 5
self.CAKE_FIRST = True  # 想退回旧顺序就改成 False（方便真机对比）

if self.CAKE_FIRST and total_drop_times == 0:
    item_type = self.arm.grasp1()       # 第一块就上二层
...
if item_type == self.CAKE_ID:
    drop_zone = "Collection_A"          # 蛋糕固定 A
elif self.CAKE_FIRST and total_drop_times == 0:
    drop_zone = "Collection_A"          # 第一块没认成蛋糕也先放 A（避免占掉 B/C/D）
    rospy.logwarn(...)
else:
    drop_zone = self.mapping.get(item_type, self.default_zone)
```

两点提醒给 D 组：

1. **第一块就上二层**意味着开局位姿必须能直接吸到二层中间那块；如果 `grasp1()` 依赖某些前置状态
   （比如机械臂高度、导航点），需要在真机上确认；
2. 第一块若不是蛋糕（模型缺 `cake` 类时必然如此），代码会**照样送 A**，用 warn 记一笔；
   这样至少不会把最贵的那一格 B/C/D 占掉，但蛋糕本身仍然拿不到分 —— 根子还是权重缺类。

## 9. 上机前先查权重类别

```bash
python3 tools/inspect_model.py            # 默认查 ~/yolov8n.pt
```

在 `yolo_venv` 中运行，优先使用 Ultralytics，旧 YOLOv5 权重会回退到 torch 读取，输出形如：

```
== /home/spark/yolov8n.pt ==
    0  bear
    1  wine
    2  clock
   共 3 类: ['bear', 'clock', 'wine']

=== 规则需要的类别自检 ===
   OK  bear
   缺! clock ... （示意）
```

结论里会直接告诉你缺哪类、以及「cake 缺失该怎么临时绕过」。


## 10. 部署到机器人（无 git，用 scp/rsync）

机器人上**没有**本仓库的工作副本（现役代码是 `~/spark_noetic/src/3rd_app/auto_match_25/auto_match`），
所以不要指望 `git pull`。部署流程（先备份、只覆盖视觉文件）：

```bash
HOST=spark@10.24.170.0
DEST=/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match
BASE=$(date +%Y%m%d_%H%M%S)

# 1) 备份（整包，出事能整体回滚）
ssh $HOST "cp -a $DEST ~/backup_auto_match_$BASE"

# 2) 只覆盖视觉相关文件
cd auto_match_25/auto_match
scp nodes/detect_ros.py nodes/board_mapping.py nodes/board_classifier.py $HOST:$DEST/nodes/
scp config/items_config.yaml                     $HOST:$DEST/config/
scp scripts/run_detect_ros.sh                    $HOST:$DEST/scripts/
scp launch/vision_test.launch                    $HOST:$DEST/launch/

# 3) 补执行位（scp 不带权限位）
ssh $HOST "chmod +x $DEST/nodes/board_classifier.py $DEST/nodes/board_mapping.py $DEST/scripts/run_detect_ros.sh"

# 4) 验证视觉链路（不启底盘/机械臂）
ssh $HOST "source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash && \
  roslaunch auto_match vision_test.launch start_camera:=true" &
sleep 12
ssh $HOST "source /opt/ros/noetic/setup.bash && source ~/spark_noetic/devel/setup.bash && \
  rosservice call /board_classifier/capture && rostopic hz /objects --window=20" 
```

回滚：`ssh $HOST "rm -rf $DEST && cp -a ~/backup_auto_match_$BASE $DEST"`

> 只部署视觉文件是**故意**的：`runtask.py` / `nav_manager.py` 依赖 B 组点位（`Collection_A` 仍未标），
> 整包部署会把别人正在调的导航一起换掉。

## 11. 和另外三个人怎么不撞车

* A 独占：`nodes/detect_ros.py`、`config/items_config.yaml`、`nodes/board_*.py`、`test/`、`docs/vision_module.md`、`tools/inspect_model.py`。
* `runtask.py` 现在是 8 处改动：6 处分类板对接 + 蛋糕优先（`self.CAKE_ID` / `self.CAKE_FIRST` + 两处判断）。
  全部带注释，`CAKE_FIRST = False` 即可退回旧顺序。
* 不在这几个文件里 import D 的 `RobotMoveAction`，保持单向依赖（D → A）。
