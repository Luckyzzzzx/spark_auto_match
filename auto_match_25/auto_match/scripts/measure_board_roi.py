#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在分类区停车位跑一次，自动量出分类板 ROI 并写回 items_config.yaml。

为什么要标 ROI：分类板和中央金砖区用的是同样三种图案（玩偶/时钟/香蕉），
同一个相机如果同时看到两者，board_classifier 可能把金砖当成板子。
ROI 把搜索范围限制在板子所在的矩形内，是最后一道防线。

用法（机器人停在 Classification_area 之后）：

    # 1) 只测量并打印，不改文件
    rosrun auto_match measure_board_roi.py --seconds 8

    # 2) 满意后直接写回 config/items_config.yaml（会先备份 .bak）
    rosrun auto_match measure_board_roi.py --seconds 8 --write

    # 3) 额外放宽/收紧四周余量（像素）
    rosrun auto_match measure_board_roi.py --margin 30

判读：
    * 采到 20 帧以上且 accepted 比例高 -> 结果可信；
    * 一直 "目标不足/尺寸不一致" -> 板子没在画面里，或者停车位偏了；
    * 写回后建议再跑一次 /board_classifier/capture 验证 mapping。
"""

import argparse
import io
import os
import re
import sys

import rospy
import rospkg
import yaml
from vision_msgs.msg import Detection2DArray

from board_mapping import (find_duplicate_ids, gap_uniform_ratio,
                           normalize_detections, size_uniform_ratio)

CONFIG_REL = os.path.join("config", "items_config.yaml")


def load_all():
    path = os.path.join(rospkg.RosPack().get_path("auto_match"), CONFIG_REL)
    with io.open(path, "r", encoding="utf8") as fp:
        return path, (yaml.safe_load(fp.read()) or {})


def resolve_ids(names, items):
    return [int(items[n]) for n in (names or []) if n in items]


def frame_detections(msg):
    """Detection2DArray -> [(id, cx, cy, w, h, score), ...]（原始像素信息）"""
    dets = []
    for obj in msg.detections:
        if not obj.results:
            continue
        hyp = obj.results[0]
        dets.append((hyp.id, float(obj.bbox.center.x), float(obj.bbox.center.y),
                     float(obj.bbox.size_x), float(obj.bbox.size_y), float(hyp.score)))
    return dets


def frame_items(raw, include_ids, ignore_ids, x_tolerance):
    """交给 board_mapping 的 5 元组（id, cx, score, w, h），并做白名单/去重。"""
    return normalize_detections(
        [(r[0], r[1], r[5], r[3], r[4]) for r in raw],
        ignore_ids, x_tolerance, include_ids)


def collect(seconds, include_ids, ignore_ids, x_tolerance, expect_num,
            min_gap, min_size):
    """采若干秒，返回所有通过校验的帧 [(x1,y1,x2,y2, items), ...] 及统计。"""
    good = []
    stats = {"frames": 0, "incomplete": 0, "extra": 0, "duplicate": 0,
             "geometry": 0, "size": 0}

    def cb(msg):
        stats["frames"] += 1
        raw = frame_detections(msg)
        items = frame_items(raw, include_ids, ignore_ids, x_tolerance)
        if len(items) < expect_num:
            stats["incomplete"] += 1
            return
        if len(items) > expect_num:
            stats["extra"] += 1
            return
        if find_duplicate_ids(items):
            stats["duplicate"] += 1
            return
        if min_gap > 0 and gap_uniform_ratio(items, expect_num) < min_gap:
            stats["geometry"] += 1
            return
        if min_size > 0 and size_uniform_ratio(items, expect_num) < min_size:
            stats["size"] += 1
            return
        # 用原始像素信息算外接矩形（normalize 之后只剩 x/w/h，没有 cy）
        by_id = {}
        for r in raw:
            by_id.setdefault(r[0], r)
        xs, ys = [], []
        for item in items:
            r = by_id.get(item[0])
            if r is None:
                continue
            _, cx, cy, w, h, _score = r
            xs.extend([cx - w / 2.0, cx + w / 2.0])
            ys.extend([cy - h / 2.0, cy + h / 2.0])
        if not xs or not ys:
            stats["incomplete"] += 1
            return
        good.append((min(xs), min(ys), max(xs), max(ys), items))

    sub = rospy.Subscriber("/objects", Detection2DArray, cb, queue_size=5)
    rospy.loginfo("开始采集 %.1f 秒，请让分类板保持在画面中…", seconds)
    rospy.sleep(seconds)
    sub.unregister()
    return good, stats


def suggest_roi(good, margin):
    """用通过校验的帧取「多数重叠区间」，比直接取并集更抗离群值。"""
    xs1 = sorted(frame[0] for frame in good)
    ys1 = sorted(frame[1] for frame in good)
    xs2 = sorted(frame[2] for frame in good)
    ys2 = sorted(frame[3] for frame in good)

    def pick(values, high):
        # 取 10%~90% 分位，去掉两端离群帧
        lo = int(len(values) * 0.1)
        hi = max(lo, int(len(values) * 0.9) - 1)
        return values[hi] if high else values[lo]

    x1 = int(pick(xs1, False)) - margin
    y1 = int(pick(ys1, False)) - margin
    x2 = int(pick(xs2, True)) + margin
    y2 = int(pick(ys2, True)) + margin
    return [max(0, x1), max(0, y1), x2, y2]


def write_roi(config_path, roi):
    """把 roi 写回 items_config.yaml（只替换 board.roi 那一行，保留注释）。"""
    text = io.open(config_path, "r", encoding="utf8").read()
    backup = config_path + ".bak"
    io.open(backup, "w", encoding="utf8").write(text)
    new_line = "  roi: [%d, %d, %d, %d]" % tuple(roi)
    new_text, count = re.subn(r"^  roi:.*$", new_line, text, count=1, flags=re.M)
    if count == 0:
        raise SystemExit("在 %s 里没找到 '  roi:' 行，请手动添加：%s" % (config_path, new_line))
    io.open(config_path, "w", encoding="utf8").write(new_text)
    return backup, new_line


def main():
    parser = argparse.ArgumentParser(description="测量分类板 ROI 并写回 items_config.yaml")
    parser.add_argument("--seconds", type=float, default=8.0, help="采集时长（秒）")
    parser.add_argument("--margin", type=int, default=25, help="四周余量（像素，默认 25）")
    parser.add_argument("--write", action="store_true", help="写回配置文件（自动备份 .bak）")
    args = parser.parse_args()

    rospy.init_node("measure_board_roi", anonymous=True)
    config_path, config = load_all()
    items = config.get("items") or {}
    board = config.get("board") or {}

    include_ids = resolve_ids(board.get("include_names"), items)
    if not include_ids:
        raise SystemExit("board.include_names 没配置（或 items 里找不到对应类别）")
    ignore_ids = board.get("ignore_ids") or []
    x_tolerance = float(board.get("x_tolerance", 30))
    expect_num = int(board.get("expect_num", 3))
    min_gap = float(board.get("min_gap_ratio", 0.0) or 0.0)
    min_size = float(board.get("min_size_ratio", 0.0) or 0.0)

    rospy.loginfo("白名单=%s 忽略=%s 期望数量=%d 间距下限=%.2f 尺寸下限=%.2f",
                  include_ids, ignore_ids, expect_num, min_gap, min_size)

    good, stats = collect(args.seconds, include_ids, ignore_ids, x_tolerance,
                          expect_num, min_gap, min_size)

    total = max(1, stats["frames"])
    print()
    print("== 采集统计 ==")
    print("  总帧数 %d，通过校验 %d（%.0f%%）"
          % (stats["frames"], len(good), 100.0 * len(good) / total))
    for key in ("incomplete", "extra", "duplicate", "geometry", "size"):
        if stats[key]:
            print("  丢弃 %-10s %d 帧" % (key, stats[key]))

    if len(good) < 10:
        print("\n[FAIL] 有效帧不足 10 帧，不生成 ROI。")
        print("  排查：板子是否在画面里？停车位是否太偏？min_size_ratio 是否过严？")
        return 1

    roi = suggest_roi(good, args.margin)
    print("\n== 建议 ROI（像素，x1, y1, x2, y2）==")
    print("  roi: [%d, %d, %d, %d]" % tuple(roi))
    print("  边长 %d x %d px，四周余量 %d px" % (roi[2] - roi[0], roi[3] - roi[1], args.margin))

    if args.write:
        backup, line = write_roi(config_path, roi)
        print("\n[OK] 已写回 %s" % config_path)
        print("     写入行: %s" % line.strip())
        print("     备份: %s" % backup)
        print("     下一步: rosservice call /board_classifier/capture 验证 mapping")
    else:
        print("\n（未写回；确认无误后加 --write 再跑一次）")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
