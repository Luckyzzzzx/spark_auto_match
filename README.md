# 夺宝奇兵自动赛 · 自动任务组 · 代码基线

本仓库是从 Spark-H 机器人（自动组借用调试）上拉取的 `auto_match_25` **真实代码基线**，用于版本管理和多人协作。

> 注意：机器上的代码比官方发布的 `auto_match_25.zip` 新很多，已大改（识别换成 YOLOv5、主逻辑重写、增加二层抓取等），但存在半成品和 bug。

## 目录结构

- `auto_match_25/auto_match/` — 主功能包
  - `nodes/` 各节点（含多个历史版本 `runtask1.py`/`runtask_7_16.py` 等，**当前只使用 `runtask.py`**）
  - `config/` `items_config.yaml`、`mark_map.pkl`、地图、`move_base_param/`
  - `launch/` `auto_match.launch`
- `auto_match_25/common/` — 动作 / 消息 / 服务
- `auto_match_25/spark_alignment_mark/` — AR 码对齐节点
- `auto_match_25/spark_move/` — 直行 / 转向控制
- `tools/` — 标定数据（`thefile.txt` / `thefile2.txt`）与桌面工具脚本

## 已知问题（待完善）

1. 识别已换 YOLOv5，但 `yolov5` 包与 `auto.pt` / `banana.pt` 模型缺失。
2. `runtask.py` 里 `detect_classification_board()` 未接入主流程，`self.mapping` 未初始化 → 运行会崩。
3. `if item_id == (99 or 10 or 20 or 30)` 逻辑错误（只等于 99）。
4. 香蕉放置分支缺失。

## A 组（视觉感知）改动 · 2026-11-18

详见 [`auto_match_25/auto_match/docs/vision_module.md`](auto_match_25/auto_match/docs/vision_module.md)。

- `detect_ros.py` 重写：阈值/模型/推理参数改由 `config/items_config.yaml` 的 `detect` 段控制，
  加帧率上限、错误不再静默吞掉；`/objects` 话题契约不变。
- 新增 `nodes/board_mapping.py`（纯逻辑）+ `nodes/board_classifier.py`（ROS 节点）：
  把分类板的「图案 → B/C/D 放置区」独立成节点，发布 `/board_mapping`，不再挤在 `runtask.py` 里。
- `runtask.py` 只加了 6 处对接（初始化 `self.mapping`、到分类区后取一次映射、兜底放置区），
  并修正 `item_id == (99 or 10 or 20 or 30)` → `item_id in (10, 20, 30, 40, 99)`。
- 新增单元测试 `test/test_board_mapping.py`（不需要 ROS / 相机 / 模型）：
  `cd auto_match_25/auto_match && python3 -m unittest discover -s test -v`
- 新节点记得 `chmod +x nodes/board_classifier.py nodes/board_mapping.py`，否则 `launch` 的 `type=` 找不到。

> 上面「已知问题」第 2、3 条已处理；第 1 条（模型缺失）现在会在启动时明确报错退出，
> 第 4 条（香蕉放置分支）属于 D 组范围。

## 环境

- 机器人：Spark-H（借用），Ubuntu 20.04 + ROS Noetic
- AI 环境：conda `python3.7.1`（torch 1.11 + tensorflow 2.4.1 + opencv）
- SSH：`spark@10.25.141.8`（已配置免密）
