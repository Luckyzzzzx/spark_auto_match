# 视觉感知模块说明（A 组）

> 对应分工表第一行：`detect_ros.py`、`items_config.yaml`、分类板识别（新增）。
> 最后更新：2026-11-18

## 1. 这次改了什么

| 文件 | 状态 | 说明 |
| --- | --- | --- |
| `nodes/detect_ros.py` | 重写 | 阈值/模型/推理参数全部从 yaml 读；加帧率上限；错误不再静默吞掉；`/objects` 契约不变 |
| `config/items_config.yaml` | 扩充 | 新增 `detect` / `board` / `board_fallback_zone` 三段，原有 `items` / `objects` 原样保留 |
| `nodes/board_mapping.py` | **新增** | 分类板映射的纯逻辑（不依赖 ROS，可单测） |
| `nodes/board_classifier.py` | **新增** | ROS 节点：`/objects` → `/board_mapping`（`std_msgs/String`，JSON，latched） |
| `test/test_board_mapping.py` | **新增** | 17 个单元测试，不需要 ROS / 相机 / 模型 |
| `launch/auto_match.launch` | +3 行 | 启动 `board_classifier` 节点 |
| `nodes/runtask.py` | +6 处小改 | 与 D 组对接：初始化 `self.mapping`、到分类区后取一次映射、兜底放置区；顺手修掉 `item_id == (99 or 10 or 20 or 30)` |
| `docs/vision_module.md` | **新增** | 本文件 |

## 2. 数据流

```
/camera/color/image_raw
        │
        ▼
   detect_ros.py  ──► /objects (vision_msgs/Detection2DArray)
        │                 │
        ▼                 ▼
   /debug_image     board_classifier.py ──► /board_mapping (std_msgs/String, latched)
                                                │
                                                ▼
                                        runtask.py（D 组）
```

这样 A 组的三个文件自成闭环：`detect_ros.py` 不认识分类板，`board_classifier.py` 不认识机械臂，
两边只通过 `/objects` 说话；D 组只读 `/board_mapping`，不需要知道识别是怎么做的。

## 3. 接口契约（改动 / 新增的部分）

### 3.1 `/objects`（沿用旧契约，未改字段）

| 字段 | 含义 |
| --- | --- |
| `header` | 原始图像的时间戳/坐标系 |
| `detections[i].results[0].id` | 物品 ID（来自 `items_config.yaml` 的 `items` 段） |
| `detections[i].results[0].score` | 置信度 |
| `detections[i].bbox.center.x / y` | 框中心点（像素） |
| `detections[i].bbox.size_x / size_y` | 框宽高（像素） |

**没有过滤场地标记**（`bear_site=10` 等仍会下发），保持和旧版一致，避免影响 mark_move 等已有订阅者。
分类板节点自己用 `board.ignore_ids` 忽略这些 ID。想让 `/objects` 就干净，把 `detect.filter_ids`
填成 `[10, 20, 30, 40, 99]` 即可。

### 3.2 `/board_mapping`（新增）

`std_msgs/String`，内容是 JSON，`latch=True`（后启动的订阅者也能立刻收到最后一帧结果）：

```json
{
  "mapping": {"1": "Collection_B", "3": "Collection_C", "2": "Collection_D"},
  "zones": ["Collection_B", "Collection_C", "Collection_D"],
  "stable": true,
  "stamp": 1763456789.12
}
```

* 键是物品 ID（**字符串**，JSON 限制），值是放置区名字；
* 只有连续 `board.stable_frames` 帧结果一致才会发布；
* 板子上少于 `board.expect_num` 个目标时**不发布**，只打 warn。

下游用法（已写进 `runtask.py`）：

```python
msg = rospy.wait_for_message("/board_mapping", String, timeout=10)
mapping = {int(k): v for k, v in json.loads(msg.data)["mapping"].items()}
drop_zone = mapping.get(item_type, "Collection_B")
```

## 4. 配置速查（`config/items_config.yaml`）

| 键 | 默认 | 什么时候改 |
| --- | --- | --- |
| `detect.conf_thres` | `0.5` | 误检多 → 调高；漏检多 → 调低 |
| `detect.class_conf` | `{}` | 单个类别单独设阈值，例如 `{clock: 0.6}` |
| `detect.augment` | `true` | **想提速首选**：改 `false` 一般能快 1.5~2 倍，精度不够再改回 `true` |
| `detect.img_size` | `640` | 降到 `416/320` 也能提速，小目标（分类板图案）会变差 |
| `detect.max_rate_hz` | `8.0` | 双模型推理本来就慢，主动丢帧避免 `/objects` 越积越旧；设 `0` 表示不限 |
| `detect.publish_debug_image` | `true` | 比赛时改 `false`，省带宽和一次图像转换 |
| `detect.filter_ids` | `[]` | 想让 `/objects` 不下发场地标记就填 `[10, 20, 30, 40, 99]` |
| `board.zones` | B/C/D | 板子格子顺序变了才改（**从左到右**） |
| `board.expect_num` | `3` | 板子要求识别几个才完整 |
| `board.stable_frames` | `3` | 现场抖动大 → 调 5；比赛求快 → 1（不建议） |
| `board.x_tolerance` | `30` | 同一个框被两个模型各检出一次时去重；板上格子很近就调小 |
| `board.ignore_ids` | `[10,20,30,40,99]` | 场地标记/底板 ID，不参与映射 |
| `board.republish_period` | `2.0` | 兜底重发间隔（秒），`0` = 只靠 latch |
| `board_fallback_zone` | `Collection_B` | 分类板识别失败时的默认放置区 |

## 5. 上机步骤（ROS Noetic）

```bash
# 1) 拷文件进工作空间（只覆盖这几个文件，其它不要动）
cp auto_match_25/auto_match/nodes/detect_ros.py        ~/catkin_ws/src/auto_match/nodes/
cp auto_match_25/auto_match/nodes/board_classifier.py  ~/catkin_ws/src/auto_match/nodes/
cp auto_match_25/auto_match/nodes/board_mapping.py     ~/catkin_ws/src/auto_match/nodes/
cp auto_match_25/auto_match/config/items_config.yaml   ~/catkin_ws/src/auto_match/config/
cp auto_match_25/auto_match/nodes/runtask.py           ~/catkin_ws/src/auto_match/nodes/   # D 组确认后再覆盖
mkdir -p ~/catkin_ws/src/auto_match/test ~/catkin_ws/src/auto_match/docs
cp auto_match_25/auto_match/test/test_board_mapping.py ~/catkin_ws/src/auto_match/test/
cp auto_match_25/auto_match/docs/vision_module.md      ~/catkin_ws/src/auto_match/docs/

# 2) 加执行权限（launch 用 type= 启动，靠的就是执行位）
chmod +x ~/catkin_ws/src/auto_match/nodes/board_classifier.py
chmod +x ~/catkin_ws/src/auto_match/nodes/board_mapping.py

# 3) 编译（纯 python 节点，只是刷新环境）
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

### 验证顺序（建议按这个次序，出问题容易定位）

```bash
# a. 单测（不用 ROS，先确认逻辑没写错）
cd ~/catkin_ws/src/auto_match && python3 -m unittest discover -s test -v

# b. 只看识别
rosrun auto_match detect_ros.py
rostopic hz /objects            # 应该有频率；0Hz 说明模型没加载（看终端 rospy.logfatal）
rostopic echo /objects -n1      # 确认 id/score/bbox 有值

# c. 对着分类板看映射
rosrun auto_match board_classifier.py
rostopic echo /board_mapping    # 板子摆好后应出现 JSON；不稳就调 stable_frames
# 台架上想"出结果就退出"：
rosrun auto_match board_classifier.py _single_shot:=true

# d. 整套
roslaunch auto_match auto_match.launch
```

判读要点：

* `detect_ros.py` 起不来并且报 `没有任何可用模型` → 检查 `~/auto.pt`、`~/banana.pt` 是否真的存在；
* `/objects` 有频率但 `detections` 一直是空 → 看 `rostopic echo /debug_image`（或 rviz）画的框，
  大概率是 `conf_thres` 太高或模型类别名和 `items` 里的键不一致（类别名必须是 `bear/wine/clock/banana`）；
* `/board_mapping` 一直不出现 → 板子没摆正、图案没被检出（`board.expect_num` 是 3），看 warn 日志里的
  「本帧 N 个」。

## 6. 已知风险 / 前提

1. **模型文件不在仓库里**（`auto.pt`、`banana.pt`），旧代码也是从 `$HOME` 读。现在缺失会在启动时报
   `rospy.logfatal` 并 `exit(2)`，不会再"看起来在跑其实什么都没检测"。
2. `vision_msgs` 的 `ObjectHypothesisWithPose.id` 在 Noetic 是整型；如果你们装的版本是字符串型，
   `board_mapping.normalize_detections` 里的 `int(item_id)` 也能兜住，但 `detect_ros.py` 里
   `hypothesis.id = item_id` 需要改成 `str(item_id)`，改一行即可。
3. 分类板节点的 `single_shot`、`stable_frames` 都可通过 rosparam 覆盖，不用改代码：
   `rosrun auto_match board_classifier.py _stable_frames:=5`。
4. `runtask.py` 里旧的 `detect_classification_board()` 保留没删（兜底用），但它的 `(99 or 10 or 20 or 30)`
   已修正为 `in (10, 20, 30, 40, 99)`。确认新流程稳定后可以整段删掉。

## 7. 和另外三个人怎么不撞车

* A 独占：`nodes/detect_ros.py`、`config/items_config.yaml`、`nodes/board_*.py`、`test/`、`docs/vision_module.md`。
* `runtask.py` 是 D 的文件：这次只加了 6 处（import、`__init__` 初始化、两个小方法、调用一次、兜底区），
  都集中在一处，D 可以自己 merge，也可以整段不要——不要的话 `/board_mapping` 只是没人用，
  A 的模块本身仍然能单独跑、单独验证。
* 不要在这几个文件里 import D 的 `RobotMoveAction`、也不要直接调 `self.robot`，保持单向依赖（D → A）。
