#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分类板识别节点（A 组新增模块）。

职责边界
--------
输入：`/objects`（vision_msgs/Detection2DArray，detect_ros.py 发布）
输出：`/board_mapping`（std_msgs/String，JSON，latched）

    {"mapping": {"1": "Collection_B", "3": "Collection_C", "2": "Collection_D"},
     "zones": ["Collection_B", "Collection_C", "Collection_D"],
     "stable": true,
     "stamp": 1234.5}

下游（D 组 runtask.py）只需要：

    from std_msgs.msg import String
    msg = rospy.wait_for_message("/board_mapping", String, timeout=10)
    mapping = {int(k): v for k, v in json.loads(msg.data)["mapping"].items()}

这样 runtask.py 不用再自己遍历检测结果，A 也能单独调分类板识别，
不会和 D 抢 `runtask.py` 的文件所有权。

运行
----
    rosrun auto_match board_classifier.py
    rosrun auto_match board_classifier.py _single_shot:=true   # 出结果就退出，方便台架调试
"""

import json
import os
import sys

import yaml

import rospy
import rospkg
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from board_mapping import MappingVoter, DEFAULT_ZONES, find_duplicate_ids, normalize_detections


def load_config():
    """读 items_config.yaml；找不到就退回默认值，保证节点还能跑。"""
    default = {
        "enabled": True,
        "zones": list(DEFAULT_ZONES),
        "expect_num": len(DEFAULT_ZONES),
        "stable_frames": 3,
        "x_tolerance": 30,
        "ignore_ids": [10, 20, 30, 40, 99],
        "frame_timeout": 5.0,
        "republish_period": 2.0,
    }
    path = None
    try:
        path = os.path.join(rospkg.RosPack().get_path("auto_match"), "config", "items_config.yaml")
    except Exception as exc:  # rospkg 找不到包时（例如在笔记本上试跑）
        rospy.logwarn("找不到 auto_match 包路径，使用默认配置: %s", exc)

    if path and os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf8") as fp:
                content = yaml.safe_load(fp.read()) or {}
        except Exception as exc:
            rospy.logerr("读取 %s 失败，使用默认配置: %s", path, exc)
            return default
        board_cfg = content.get("board") or {}
        cfg = dict(default)
        cfg.update({k: v for k, v in board_cfg.items() if v is not None})
        rospy.loginfo("分类板配置来自 %s: %s", path, cfg)
        return cfg
    return default


def to_tuples(obj_array):
    """Detection2DArray -> [(item_id, center_x, score), ...]"""
    detections = []
    for obj in obj_array.detections:
        if not obj.results:
            continue
        hypothesis = obj.results[0]
        detections.append((hypothesis.id, obj.bbox.center.x, hypothesis.score))
    return detections


class BoardClassifier(object):
    def __init__(self):
        cfg = load_config()

        self.enabled = bool(cfg.get("enabled", True))
        self.zones = list(cfg["zones"])
        self.frame_timeout = float(cfg["frame_timeout"])
        self.republish_period = float(cfg["republish_period"])
        self.single_shot = bool(rospy.get_param("~single_shot", False))

        if not self.enabled:
            # 配置里关掉时节点空转，方便临时绕过分类板（runtask 会用兜底放置区）
            rospy.logwarn("board.enabled=false，分类板识别已关闭")
            self.pub = rospy.Publisher("/board_mapping", String, queue_size=1, latch=True)
            return

        self.voter = MappingVoter(
            zones=self.zones,
            expect_num=int(cfg["expect_num"]),
            stable_frames=int(cfg["stable_frames"]),
            x_tolerance=float(cfg["x_tolerance"]),
            ignore_ids=cfg["ignore_ids"],
        )

        self.mapping = {}
        self.frames_seen = 0
        self.last_frame_time = rospy.Time.now()
        self.last_published = None

        self.pub = rospy.Publisher("/board_mapping", String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/objects", Detection2DArray, self.objects_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.watchdog)
        rospy.loginfo("board_classifier 就绪：zones=%s expect_num=%s stable_frames=%s",
                      self.zones, self.voter.expect_num, self.voter.stable_frames)

    # ------------------------------------------------------------------ 回调
    def objects_cb(self, msg):
        self.frames_seen += 1
        self.last_frame_time = rospy.Time.now()

        detections = to_tuples(msg)
        items = normalize_detections(detections, self.voter.ignore_ids, self.voter.x_tolerance)
        duplicated = find_duplicate_ids(items)
        if duplicated:
            rospy.logwarn_throttle(2.0, "分类板同一图案出现多次，映射可能不准: ids=%s", duplicated)

        mapping, state = self.voter.update(detections)
        if state == "stable":
            self.mapping = mapping
            self.publish(mapping, msg.header.stamp)
            rospy.loginfo("分类板识别稳定: %s", mapping)
            if self.single_shot:
                rospy.loginfo("single_shot=true，退出")
                rospy.signal_shutdown("single_shot done")
        elif state == "incomplete":
            rospy.logwarn_throttle(
                2.0, "分类板目标不足：本帧 %d 个，需要 %d 个，继续等待",
                len(items), self.voter.expect_num)
        elif state == "unstable":
            rospy.loginfo_throttle(2.0, "分类板识别中（未稳定）: %s", items)

    def watchdog(self, _event):
        if self.republish_period > 0 and self.last_published is not None:
            elapsed = (rospy.Time.now() - self.last_published[1]).to_sec()
            if elapsed >= self.republish_period:
                self.publish(self.last_published[0], rospy.Time.now())

        idle = (rospy.Time.now() - self.last_frame_time).to_sec()
        if idle > self.frame_timeout:
            rospy.logwarn_throttle(
                5.0, "/objects 已经 %.1fs 没有新消息，检查 detect_ros 是否在跑", idle)

    # ------------------------------------------------------------------ 发布
    def publish(self, mapping, stamp):
        payload = {
            "mapping": {str(int(k)): str(v) for k, v in mapping.items()},
            "zones": self.zones,
            "stable": True,
            "stamp": stamp.to_sec() if hasattr(stamp, "to_sec") else 0.0,
        }
        self.pub.publish(String(json.dumps(payload, ensure_ascii=False)))
        self.last_published = (mapping, rospy.Time.now())


def main():
    rospy.init_node("board_classifier")
    BoardClassifier()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
