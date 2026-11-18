#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分类板 -> B/C/D 映射的纯逻辑。

这个文件刻意 **不 import rospy**，只做数据变换：
    检测结果 [(id, x, score), ...]  ->  {item_id: 'Collection_B', ...}

好处：
  1. 可以在没有 ROS 的电脑上跑单元测试（test/test_board_mapping.py）；
  2. board_classifier.py（ROS 节点）只负责收发消息，逻辑改这里不影响 ROS 层；
  3. 以后想把分类板识别并进 detect_ros.py，直接复用这些函数即可。
"""

# 分类板三个格子，从左到右对应的放置区
DEFAULT_ZONES = ("Collection_B", "Collection_C", "Collection_D")


def normalize_detections(detections, ignore_ids=(), x_tolerance=0):
    """把一帧里的检测结果清洗成「从左到右、每格一个」。

    :param detections: 可迭代的 (item_id, center_x, score)。
                       也接受 (item_id, center_x) 两元组，score 记为 0。
    :param ignore_ids: 需要丢弃的 ID（场地标记、底板等）。
    :param x_tolerance: 中心点 x 距离小于该值视为同一个物体，只留分数最高的。
                        0 表示不做去重。
    :return: [(item_id, x, score), ...]，按 x 从小到大排序。
    """
    ignore = set(ignore_ids or ())
    candidates = []
    for det in detections:
        if len(det) >= 3:
            item_id, x, score = det[0], det[1], det[2]
        else:
            item_id, x = det[0], det[1]
            score = 0.0
        try:
            item_id = int(item_id)
            x = float(x)
            score = float(score)
        except (TypeError, ValueError):
            continue
        if item_id in ignore:
            continue
        candidates.append((item_id, x, score))

    # 先按分数从高到低，保证同一格只保留最可信的那个
    candidates.sort(key=lambda item: item[2], reverse=True)

    kept = []
    for item_id, x, score in candidates:
        duplicated = False
        if x_tolerance and x_tolerance > 0:
            for _kept_id, kept_x, _kept_score in kept:
                if abs(kept_x - x) <= x_tolerance:
                    duplicated = True
                    break
        if duplicated:
            continue
        kept.append((item_id, x, score))

    kept.sort(key=lambda item: item[1])
    return kept


def build_mapping(items, zones=DEFAULT_ZONES):
    """按「从左到右」的顺序把物品 ID 映射到放置区。

    :param items: normalize_detections 的输出（已按 x 升序）。
    :param zones: 从左到右的放置区列表。
    :return: {item_id: zone}。物品比 zone 多时，多出来的忽略。
    """
    mapping = {}
    for zone, item in zip(zones, items):
        item_id = item[0]
        if item_id in mapping:
            # 板上同一个图案出现两次 -> 信息不足，保留第一个，由上层决定是否告警
            continue
        mapping[item_id] = zone
    return mapping


def find_duplicate_ids(items):
    """返回在同一帧里重复出现的 item_id 列表（重复 = 无法确定放在哪一格）。"""
    seen = set()
    duplicated = []
    for item in items:
        item_id = item[0]
        if item_id in seen and item_id not in duplicated:
            duplicated.append(item_id)
        seen.add(item_id)
    return duplicated


class MappingVoter(object):
    """多帧投票：只有连续 stable_frames 帧结果一致，才认为分类板识别可信。

    用法::

        voter = MappingVoter(stable_frames=3, x_tolerance=30)
        for detections in frames:
            mapping, state = voter.update(detections)
            if state == "stable":
                publish(mapping)
    """

    def __init__(self, zones=DEFAULT_ZONES, expect_num=None, stable_frames=1,
                 x_tolerance=0, ignore_ids=()):
        self.zones = tuple(zones)
        # 默认要求至少填满所有格子
        self.expect_num = int(expect_num) if expect_num else len(self.zones)
        self.stable_frames = max(1, int(stable_frames))
        self.x_tolerance = x_tolerance
        self.ignore_ids = tuple(ignore_ids or ())
        self.mapping = None
        self._history = []

    def reset(self):
        """清空历史（比如场景变了、机器人移动之后）。"""
        self.mapping = None
        self._history = []

    def update(self, detections):
        """喂一帧检测结果。

        :return: (mapping, state)
            state == "incomplete" 本帧目标数量不够，不参与投票
            state == "unstable"   本帧有效但还没稳定
            state == "stable"     本次刚稳定，mapping 是结果 -> 应该发布
            state == "unchanged"  已稳定的结果没变化 -> 不必重复发布
        """
        items = normalize_detections(detections, self.ignore_ids, self.x_tolerance)
        if len(items) < self.expect_num:
            return None, "incomplete"

        # 多余的（第 4 个及以上）不参与，按 x 截断
        items = items[:len(self.zones)]
        mapping = build_mapping(items, self.zones)
        signature = tuple(sorted(mapping.items()))

        self._history.append(signature)
        if len(self._history) > self.stable_frames:
            del self._history[0]
        if len(self._history) < self.stable_frames:
            return None, "unstable"
        for old in self._history:
            if old != signature:
                return None, "unstable"

        if mapping == self.mapping:
            return mapping, "unchanged"
        self.mapping = mapping
        return mapping, "stable"


def zone_for(mapping, item_id, default="Collection_B"):
    """查某个物品该去哪个放置区；查不到就返回兜底区。"""
    if not mapping:
        return default
    try:
        item_id = int(item_id)
    except (TypeError, ValueError):
        return default
    return mapping.get(item_id, default)
