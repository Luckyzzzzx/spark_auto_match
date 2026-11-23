#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task_rules 的单元测试（不需要 ROS / 相机 / 场地）。

跑法（在包目录下）：
    python3 -m unittest discover -s test -t test -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))

from task_rules import (  # noqa: E402
    DEFAULT_FALLBACK,
    choose_drop_zone,
    mapping_is_usable,
    remaining_seconds,
    should_stop,
)


class ChooseZoneTest(unittest.TestCase):
    def test_cake_always_goes_a(self):
        # 即使分类板映射里写了别处，蛋糕也必须去 A（规则硬性要求）
        self.assertEqual(choose_drop_zone(5, {5: 'Collection_C'}, cake_id=5), 'Collection_A')

    def test_uses_mapping_when_present(self):
        mapping = {1: 'Collection_D', 3: 'Collection_B', 4: 'Collection_C'}
        self.assertEqual(choose_drop_zone(1, mapping), 'Collection_D')
        self.assertEqual(choose_drop_zone(4, mapping), 'Collection_C')

    def test_falls_back_per_item_when_mapping_missing(self):
        # 分类板识别失败：不能全塞 B 区，要按物品分散
        zones = {choose_drop_zone(iid, {}) for iid in DEFAULT_FALLBACK}
        self.assertGreater(len(zones), 1, "兜底区必须分散，不能全是一个区")
        self.assertEqual(choose_drop_zone(1, {}), 'Collection_B')
        self.assertEqual(choose_drop_zone(4, {}), 'Collection_D')

    def test_partial_mapping_falls_back_only_for_missing(self):
        mapping = {1: 'Collection_D'}          # 只识别出一类
        self.assertEqual(choose_drop_zone(1, mapping), 'Collection_D')
        self.assertEqual(choose_drop_zone(4, mapping), DEFAULT_FALLBACK[4])

    def test_unknown_item_uses_default(self):
        self.assertEqual(choose_drop_zone(99, {}, default_zone='Collection_C'), 'Collection_C')

    def test_bad_item_id(self):
        self.assertEqual(choose_drop_zone('bad', {}, default_zone='Collection_B'), 'Collection_B')

    def test_custom_fallback_overrides(self):
        self.assertEqual(choose_drop_zone(1, {}, {1: 'Collection_D'}), 'Collection_D')


class MappingUsableTest(unittest.TestCase):
    def test_complete(self):
        self.assertTrue(mapping_is_usable({1: 'B', 3: 'C', 4: 'D'}, [1, 3, 4]))

    def test_missing_one(self):
        self.assertFalse(mapping_is_usable({1: 'B', 3: 'C'}, [1, 3, 4]))

    def test_empty(self):
        self.assertFalse(mapping_is_usable({}, [1, 3, 4]))


class DeadlineTest(unittest.TestCase):
    def test_remaining(self):
        self.assertEqual(remaining_seconds(100.0, 40.0), 60.0)
        self.assertEqual(remaining_seconds(None, 40.0), float('inf'))

    def test_should_stop_before_deadline(self):
        # 还剩 60s，要求留 90s -> 应停止
        self.assertTrue(should_stop(940.0, 1000.0, 90.0))

    def test_should_continue_when_time_enough(self):
        self.assertFalse(should_stop(800.0, 1000.0, 90.0))

    def test_stop_after_deadline(self):
        self.assertTrue(should_stop(1001.0, 1000.0, 0.0))


if __name__ == '__main__':
    unittest.main()
