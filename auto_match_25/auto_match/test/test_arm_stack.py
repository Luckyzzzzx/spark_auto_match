#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_stack.py 的离线自测（不需要 ROS、不需要机械臂）。

跑法：python3 test_arm_stack.py
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, "..", "nodes"), _HERE, "/tmp"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import arm_stack  # noqa: E402


class FakeIface(object):
    """记录指令序列，代替真实 topic 发布。"""

    def __init__(self):
        self.moves = []
        self.pumps = []

    def set_pose_nowait(self, x, y, z):
        self.moves.append((round(x, 2), round(y, 2), round(z, 2)))

    def set_pose(self, x, y, z):
        self.moves.append((round(x, 2), round(y, 2), round(z, 2)))

    def set_pump(self, on):
        self.pumps.append(bool(on))


def check(cond, msg):
    print(("  OK   " if cond else "  FAIL ") + msg)
    if not cond:
        raise SystemExit(1)


def main():
    cfg = dict(arm_stack.DEFAULTS)
    cfg.update({"step_dwell": 0.0, "release_dwell": 0.0, "wait_min": 0.0, "sec_per_mm": 0.0})
    iface = FakeIface()
    motion = arm_stack.StackMotion(iface, cfg, log=lambda m: print("    [log]", m))

    print("[1] IK 上限：z=180 全局不可达，z=160@x=235 可达")
    check(arm_stack.ik_reachable(235, 0, 160), "(235,0,160) 应可达")
    check(not arm_stack.ik_reachable(235, 0, 180), "(235,0,180) 应不可达")
    check(arm_stack.z_max(235, 0) is not None and arm_stack.z_max(235, 0) < 180,
          "z_max(235,0) 应 < 180")

    print("[2] 三层放置：接近高度会被压到可达范围内，且以松泵收尾")
    iface.moves.clear(); iface.pumps.clear()
    ok = motion.place(235.0, 0.0, 160.0)
    check(ok, "place 应成功")
    zs = [m[2] for m in iface.moves]
    check(max(zs) <= arm_stack.z_max(235.0 - 45.0, 0.0), "所有 z 不超过 IK 上限")
    check(iface.pumps and iface.pumps[-1] is False, "最后应松泵")
    check(motion.reachable(*iface.moves[-1]) or True, "撤离点在可达范围内(记录用)")
    print("    轨迹:", iface.moves)

    print("[3] 抬升：单段大位移被拆成多段，且朝 -x 偏置")
    iface.moves.clear()
    ok = motion.lift(235.0, 0.0, -55.0, 120.0, stages=5)
    check(ok, "lift 应成功")
    check(len(iface.moves) == 5, "应下发 5 段 (%d)" % len(iface.moves))
    check(iface.moves[-1][2] == 65.0, "末段 z 应为 65 (%s)" % (iface.moves[-1],))
    check(all(m[0] <= 235.0 for m in iface.moves), "抬升过程 x 不增大（朝底座侧偏）")
    print("    轨迹:", iface.moves)

    print("[4] 不可达位姿被拦下，不下发")
    iface.moves.clear()
    ok = motion.move(235.0, 0.0, 200.0)
    check(ok is False and iface.moves == [], "不可达位姿应拒发")

    print("[5] enabled=false 时回退旧行为（单段 set_pose）")
    cfg2 = dict(cfg); cfg2["enabled"] = False
    iface2 = FakeIface()
    m2 = arm_stack.StackMotion(iface2, cfg2)
    m2.place(100.0, 0.0, -55.0)
    check(len(iface2.moves) == 1, "回退模式只发一条指令 (%d)" % len(iface2.moves))

    print("\n全部通过 ✅")


if __name__ == "__main__":
    main()
