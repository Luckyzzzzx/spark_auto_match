#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""board_mapping 的单元测试（不需要 ROS、不需要相机、不需要模型）。

跑法（在仓库根目录）：
    python3 -m unittest discover -s auto_match_25/auto_match/test -v

它验证的是分类板映射的核心逻辑：
  * 从左到右排序
  * 场地标记 ID 被忽略
  * 同一格重复检测去重
  * 多帧投票才输出稳定结果
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))

from board_mapping import (  # noqa: E402
    MappingVoter,
    build_mapping,
    find_duplicate_ids,
    normalize_detections,
    zone_for,
)


class NormalizeTest(unittest.TestCase):
    def test_sorted_left_to_right(self):
        dets = [(1, 300, 0.9), (2, 120, 0.8), (3, 210, 0.7)]
        self.assertEqual([d[0] for d in normalize_detections(dets)], [2, 3, 1])

    def test_ignore_ids(self):
        dets = [(1, 100, 0.9), (99, 200, 0.9), (10, 300, 0.9)]
        self.assertEqual([d[0] for d in normalize_detections(dets, ignore_ids=[10, 99])], [1])

    def test_dedupe_keeps_highest_score(self):
        dets = [(1, 100, 0.6), (3, 110, 0.9)]
        kept = normalize_detections(dets, x_tolerance=30)
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0][0], 3)

    def test_dedupe_disabled(self):
        dets = [(1, 100, 0.6), (3, 110, 0.9)]
        self.assertEqual(len(normalize_detections(dets, x_tolerance=0)), 2)

    def test_accepts_pairs_and_bad_values(self):
        dets = [(1, 50), ('bad', 10), (2, 'x'), (3, 80, '0.4')]
        kept = normalize_detections(dets)
        self.assertEqual([d[0] for d in kept], [1, 3])


class BuildMappingTest(unittest.TestCase):
    def test_order_matches_zones(self):
        items = [(3, 120, 0.9), (1, 200, 0.9), (2, 280, 0.9)]
        mapping = build_mapping(items)
        self.assertEqual(mapping, {3: 'Collection_B', 1: 'Collection_C', 2: 'Collection_D'})

    def test_extra_items_ignored(self):
        items = [(3, 1, 1), (1, 2, 1), (2, 3, 1), (4, 4, 1)]
        self.assertEqual(len(build_mapping(items)), 3)

    def test_duplicate_id_keeps_first(self):
        # 第 2 格误检成和第 1 格同类：位置仍然可信，所以 2 号物品按第 3 格 -> Collection_D
        items = [(1, 100, 0.9), (1, 200, 0.9), (2, 300, 0.9)]
        self.assertEqual(build_mapping(items), {1: 'Collection_B', 2: 'Collection_D'})

    def test_find_duplicate_ids(self):
        items = [(1, 100, 0.9), (1, 300, 0.9), (2, 200, 0.9)]
        self.assertEqual(find_duplicate_ids(items), [1])


class VoterTest(unittest.TestCase):
    def setUp(self):
        self.frame = [(1, 100, 0.9), (2, 200, 0.9), (3, 300, 0.9)]

    def test_needs_stable_frames(self):
        voter = MappingVoter(stable_frames=3)
        self.assertEqual(voter.update(self.frame), (None, 'unstable'))
        self.assertEqual(voter.update(self.frame), (None, 'unstable'))
        mapping, state = voter.update(self.frame)
        self.assertEqual(state, 'stable')
        self.assertEqual(mapping, {1: 'Collection_B', 2: 'Collection_C', 3: 'Collection_D'})

    def test_unchanged_after_stable(self):
        voter = MappingVoter(stable_frames=2)
        voter.update(self.frame)
        voter.update(self.frame)
        mapping, state = voter.update(self.frame)
        self.assertEqual(state, 'unchanged')
        self.assertEqual(mapping[1], 'Collection_B')

    def test_alternating_frames_never_stabilise(self):
        voter = MappingVoter(stable_frames=3)
        other = [(1, 300, 0.9), (2, 200, 0.9), (3, 100, 0.9)]
        states = []
        for frame in [self.frame, other, self.frame, other, self.frame]:
            states.append(voter.update(frame)[1])
        self.assertNotIn('stable', states)

    def test_incomplete_frame_is_ignored(self):
        voter = MappingVoter(stable_frames=2)
        voter.update(self.frame)
        self.assertEqual(voter.update([(1, 100, 0.9)]), (None, 'incomplete'))
        mapping, state = voter.update(self.frame)
        self.assertEqual(state, 'stable')
        self.assertEqual(len(mapping), 3)

    def test_reset_clears_history(self):
        voter = MappingVoter(stable_frames=2)
        voter.update(self.frame)
        voter.update(self.frame)
        voter.reset()
        self.assertEqual(voter.update(self.frame), (None, 'unstable'))

    def test_partial_board_does_not_publish(self):
        voter = MappingVoter(stable_frames=1)
        mapping, state = voter.update([(1, 100, 0.9), (2, 200, 0.9)])
        self.assertEqual((mapping, state), (None, 'incomplete'))


class ZoneForTest(unittest.TestCase):
    def test_lookup(self):
        self.assertEqual(zone_for({1: 'Collection_C'}, 1), 'Collection_C')

    def test_default_when_missing(self):
        self.assertEqual(zone_for({1: 'Collection_C'}, 2), 'Collection_B')
        self.assertEqual(zone_for(None, 2), 'Collection_B')
        self.assertEqual(zone_for({1: 'Collection_C'}, 'bad'), 'Collection_B')


if __name__ == '__main__':
    unittest.main()
