# 蛋糕纸盒识别

图片中的目标是**印有蛋糕图案的纸盒**。本项目将整个纸盒标为 `cake`，供机器人按比赛道具识别；它不是食物识别模型。

## 当前代码快照（2026-11-22）

- `new_robot_code/` 是从新机器人当前 ROS 包复制的检测、分类板和任务调度代码，供后续集成修改；这里的 `detect_ros.py` 尚未接入 `cake_best.pt`，`runtask.py` 仍使用检测框中心抓取，**不能视作已完成四角定位**。
- `previous_robot_code/` 保存旧机器人两版检测节点与类别配置；`calibration/` 是独立的顶面四角候选和机械臂标定辅助脚本。自动四角仅供检查，未经过全场景验证。
- `cake_best.pt` 是单类蛋糕 YOLOv5 权重。GitHub 代码快照不包含权重、原始相机图片或训练数据；在本地工作目录和机器人上另存。部署前须提供权重，并在目标机器人上重新验证识别、顶面中心、手眼标定和吸盘工具中心。
- 这些代码是研发快照，不应直接启动完整自动抓放流程。当前新机器人只完成模型在一帧混合方块画面上的离线推理检查，尚未实测吸盘抓取。

## 机器人现状（2026-11-22 检查）

- `/home/spark/auto.pt` 的类别为 `bear`、`wine`、`clock`、对应位置和 `base`，没有 `cake`。
- 旧试验权重 `/home/spark/cake_trial_20261122/runs/trial/weights/best.pt` 只使用 1 张训练图和 1 张验证图。对用户给出的 4 张不同图片均未正确框出纸盒。
- 机器人已有 5 次采集，每次 100 张蛋糕纸盒照片，位于 `/home/spark/transfer_out/gold_photos_cake_*`。

## 数据与训练

- `prepare_cake_dataset.py` 将用户给出的 4 张不同图片保存为 YOLO 标注格式；`cake_dataset/preview` 是人工检查用的框图。
- `build_robot_cake_dataset.py` 利用纸盒上的红色图案自动提出目标框，并按采集批次分开训练和验证。机器人上生成的 `/home/spark/cake_dataset_v2_20261122` 包含 400 张训练图、90 张验证图；10 张边缘目标被跳过。自动框需人工抽查，已检查每组第一张的预览。
- 基础训练使用机器人现有的 YOLOv5：

```bash
python3 /home/spark/.local/lib/python3.8/site-packages/yolov5/train.py \
  --weights /home/spark/auto.pt \
  --data /home/spark/cake_dataset_v2_20261122/data.yaml \
  --epochs 12 --batch-size 4 --imgsz 416 --device cpu --workers 2 \
  --project /home/spark/cake_dataset_v2_20261122/runs --name cake_v2 --exist-ok --noplots
```

- `add_user_views.py` 可在基础训练结束后复制数据集，并加入用户新拍的视角做微调。添加后的图片仍需通过独立现场画面复核。
- `add_negative_views.py` 加入香蕉、时钟、玩偶的空标注照片，避免把其他比赛道具误认成蛋糕。最终数据集位于机器人 `/home/spark/cake_dataset_v4_20261122`：780 张训练图（480 张蛋糕、300 张其他道具），150 张验证图（90 张蛋糕、60 张其他道具）。
- 最终权重：机器人 `/home/spark/cake_dataset_v4_20261122/runs/cake_v4/weights/best.pt`；本地副本 `cake_best.pt`，SHA-256 已核对一致。

最终权重在 0.5 置信度阈值、416 输入尺寸下，通过 `audit_cake_model.py` 逐张审计：90/90 张验证蛋糕图的目标框 IoU≥0.5，60/60 张验证负样本没有误报。用户提供的 4 张不同图片均正确框出，置信度约 0.72–0.79；另抽查未参与训练验证的 6 张其他道具照片，也未发现误报。该验证集仍主要来自同一地面和纸盒，不代表所有环境都能达到同样结果。

## ROS 识别节点

`cake_detect_ros.py` 订阅 `/camera/color/image_raw`，发布 `/cake_detections`（`vision_msgs/Detection2DArray`，默认目标 ID 为 5）和 `/cake_debug_image`。它不占用现有 `/objects` 话题。机器人上已经启动，可用以下命令重新运行：

```bash
bash /home/spark/start_cake_detector.sh
```

启动脚本会加载比赛工作空间的 ROS 环境。当前进程约每秒发布 4.5 次、CPU 使用率约 90%（一个核心）。节点默认使用 416 输入尺寸、0.5 置信度阈值、5 Hz 上限和 2 个 PyTorch 线程；可用私有参数 `~model_path`、`~confidence`、`~image_size`、`~object_id`、`~max_rate_hz`、`~torch_threads` 调整。接入比赛主流程时，让下游订阅 `/cake_detections`，或将 `cake` 类合并进现有 `/objects` 发布节点，并在物品映射中加入相同 ID。

**部署状态**：ROS 节点已经在机器人上运行，话题可收到带相机时间戳的消息。当前相机画面中没有蛋糕，故现场正样本仍需把纸盒放回相机视野后观察 `/cake_detections`。节点由 `nohup` 启动，机器人重启后需重新启动。
