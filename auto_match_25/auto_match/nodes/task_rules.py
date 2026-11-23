#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""调度相关的纯逻辑（不 import rospy，可以直接单测）。

解决两个会直接丢分的场景：

1. **分类板识别失败**：原来所有物品都落到 `board_fallback_zone`（默认 B 区），
   10 块金砖全堆一处 —— B 区堆满后既不得分，还可能挡住后面的正确放置。
   这里按物品分到不同兜底区，至少分散风险、不把一区堆爆。

2. **10 分钟硬限**：规则「超时放置的金砖不得分」，且超时每秒扣 2 分。
   到点必须停止**新**的抓取，把剩下的时间留给收尾。
"""

# 分类板没识别出来时，各物品的兜底放置区（按 ID）
DEFAULT_FALLBACK = {
    1: "Collection_B",    # bear
    2: "Collection_C",    # wine（旧模型才有）
    3: "Collection_C",    # clock
    4: "Collection_D",    # banana
}

CAKE_ZONE = "Collection_A"


def choose_drop_zone(item_id, mapping, fallback_zones=None, cake_id=5,
                     cake_zone=CAKE_ZONE, default_zone="Collection_B"):
    """决定某个物品该放哪个区。

    优先级：蛋糕固定 A 区 → 分类板映射 → 按物品的兜底区 → 全局兜底区。

    :param item_id: 抓到的物品 ID
    :param mapping: 分类板映射 {item_id: 'Collection_X'}，可能为空
    :param fallback_zones: 兜底映射 {item_id: 'Collection_X'}
    :param cake_id: 蛋糕 ID（规则要求固定 A 区）
    """
    try:
        item_id = int(item_id)
        cake_id = int(cake_id)
    except (TypeError, ValueError):
        return default_zone

    if item_id == cake_id:
        return cake_zone

    if mapping:
        zone = mapping.get(item_id)
        if zone:
            return str(zone)

    # 没配（None 或空字典）就用内置的按物品兜底表，保证"不会全堆一个区"
    fallback = fallback_zones or DEFAULT_FALLBACK
    zone = fallback.get(item_id)
    if zone:
        return str(zone)

    return default_zone


def mapping_is_usable(mapping, expected_ids):
    """分类板映射是否覆盖了全部应出现的物品（缺任何一个都算不可用）。"""
    if not mapping:
        return False
    try:
        return all(int(i) in mapping for i in expected_ids)
    except (TypeError, ValueError):
        return False


def remaining_seconds(deadline, now):
    """距离截止时间还剩多少秒（可为负）。"""
    if deadline is None:
        return float("inf")
    return float(deadline) - float(now)


def should_stop(now, deadline, min_remaining_s):
    """剩余时间不足 min_remaining_s 时返回 True（该停止新的抓取了）。"""
    return remaining_seconds(deadline, now) <= float(min_remaining_s)
