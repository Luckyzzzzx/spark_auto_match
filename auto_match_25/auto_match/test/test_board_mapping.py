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
    gap_uniform_ratio,
    normalize_detections,
    size_uniform_ratio,
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

    def test_incomplete_frame_breaks_consecutive_vote(self):
        voter = MappingVoter(stable_frames=2)
        voter.update(self.frame)
        self.assertEqual(voter.update([(1, 100, 0.9)]), (None, 'incomplete'))
        mapping, state = voter.update(self.frame)
        self.assertEqual((mapping, state), (None, 'unstable'))
        mapping, state = voter.update(self.frame)
        self.assertEqual(state, 'stable')
        self.assertEqual(len(mapping), 3)

    def test_geometry_frame_breaks_consecutive_vote(self):
        voter = MappingVoter(stable_frames=2, min_gap_ratio=0.45)
        voter.update(self.frame)
        self.assertEqual(voter.update([(1, 100, 0.9), (2, 110, 0.9),
                                       (3, 300, 0.9)]), (None, 'geometry'))
        self.assertEqual(voter.update(self.frame), (None, 'unstable'))

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

    def test_extra_detection_does_not_get_silently_truncated(self):
        voter = MappingVoter(stable_frames=1)
        frame = [(4, 50, 0.95), (1, 100, 0.9), (2, 200, 0.9), (3, 300, 0.9)]
        self.assertEqual(voter.update(frame), (None, 'extra'))

    def test_duplicate_class_does_not_publish_incomplete_mapping(self):
        voter = MappingVoter(stable_frames=1)
        frame = [(1, 100, 0.9), (1, 200, 0.8), (3, 300, 0.9)]
        self.assertEqual(voter.update(frame), (None, 'duplicate'))


class ZoneForTest(unittest.TestCase):
    def test_lookup(self):
        self.assertEqual(zone_for({1: 'Collection_C'}, 1), 'Collection_C')

    def test_default_when_missing(self):
        self.assertEqual(zone_for({1: 'Collection_C'}, 2), 'Collection_B')
        self.assertEqual(zone_for(None, 2), 'Collection_B')
        self.assertEqual(zone_for({1: 'Collection_C'}, 'bad'), 'Collection_B')


class BoardRulesTest(unittest.TestCase):
    """规则相关：分类板只有玩偶/时钟/香蕉，蛋糕固定 A 区，斜视要能判出来。"""

    def test_include_ids_whitelist(self):
        dets = [(1, 100, 0.9), (5, 200, 0.9), (3, 300, 0.9), (2, 400, 0.9)]
        kept = normalize_detections(dets, include_ids=[1, 3, 2])
        self.assertEqual([d[0] for d in kept], [1, 3, 2])

    def test_gap_uniform_ratio_even(self):
        items = [(1, 100, 0.9), (3, 200, 0.9), (2, 300, 0.9)]
        self.assertAlmostEqual(gap_uniform_ratio(items), 1.0)

    def test_gap_uniform_ratio_skewed(self):
        items = [(1, 100, 0.9), (3, 120, 0.9), (2, 300, 0.9)]
        self.assertLess(gap_uniform_ratio(items), 0.5)

    def test_voter_rejects_skewed_board(self):
        voter = MappingVoter(stable_frames=1, min_gap_ratio=0.45)
        skewed = [(1, 100, 0.9), (3, 115, 0.9), (2, 300, 0.9)]
        self.assertEqual(voter.update(skewed), (None, 'geometry'))

    def test_voter_accepts_view_with_mild_perspective(self):
        voter = MappingVoter(stable_frames=1, min_gap_ratio=0.45)
        mild = [(1, 100, 0.9), (3, 215, 0.9), (2, 300, 0.9)]  # 115 / 85
        mapping, state = voter.update(mild)
        self.assertEqual(state, 'stable')
        self.assertEqual(mapping[1], 'Collection_B')

    def test_cake_goes_to_a_by_fixed_rule(self):
        # 蛋糕不在板上，但固定 A 区；板上只有另外三种
        fixed = {5: 'Collection_A'}
        items = [(1, 100, 0.9), (3, 200, 0.9), (2, 300, 0.9)]
        mapping = build_mapping(items, fixed=fixed)
        self.assertEqual(mapping[5], 'Collection_A')
        self.assertEqual(mapping[1], 'Collection_B')

    def test_voter_publishes_fixed_zone_without_board_item(self):
        voter = MappingVoter(stable_frames=1, fixed={5: 'Collection_A'})
        mapping, state = voter.update([(1, 100, 0.9), (3, 200, 0.9), (2, 300, 0.9)])
        self.assertEqual(state, 'stable')
        self.assertEqual(mapping, {5: 'Collection_A', 1: 'Collection_B',
                                   3: 'Collection_C', 2: 'Collection_D'})


class SizeGuardTest(unittest.TestCase):
    """三格尺寸一致性：用来挡住"把中央 3×3 金砖当成分类板"。"""

    def _frame(self, sizes):
        # (id, x, score, w, h)
        return [(i + 1, 100 * (i + 1), 0.9, sizes[i], sizes[i]) for i in range(3)]

    def test_uniform_sizes_pass(self):
        self.assertAlmostEqual(size_uniform_ratio(self._frame([40, 42, 41])), 40 / 42.0, places=3)

    def test_mixed_sizes_fail(self):
        ratio = size_uniform_ratio(self._frame([40, 60, 20]))
        self.assertLess(ratio, 0.6)

    def test_missing_sizes_are_not_blocking(self):
        frame = [(1, 100, 0.9), (2, 200, 0.9), (3, 300, 0.9)]   # 没有 w/h
        self.assertEqual(size_uniform_ratio(frame), 1.0)

    def test_voter_rejects_size_mismatch(self):
        voter = MappingVoter(stable_frames=1, min_size_ratio=0.55)
        self.assertEqual(voter.update(self._frame([40, 60, 20])), (None, 'size'))

    def test_voter_accepts_close_sizes(self):
        voter = MappingVoter(stable_frames=1, min_size_ratio=0.55)
        mapping, state = voter.update(self._frame([40, 42, 41]))
        self.assertEqual(state, 'stable')
        self.assertEqual(mapping[1], 'Collection_B')

    def test_normalize_keeps_sizes(self):
        kept = normalize_detections([(1, 100, 0.9, 40, 30)])
        self.assertEqual(len(kept[0]), 5)
        self.assertEqual((kept[0][3], kept[0][4]), (40.0, 30.0))


if __name__ == '__main__':
    unittest.main()
