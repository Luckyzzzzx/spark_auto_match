#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""堆叠安全运动库（纯 Python，不 import rospy，可离线单测）。

为什么需要这一层
----------------
uArm Swift Pro 的 `G0` 是**关节插补**，不是直线插补：从 (x, y, z) 抬到
(x, y, z+120) 时，末端实际走的是弧线，水平方向会漂几毫米到十几毫米。
在 3x3 的密集金砖阵列里，这点漂移足够把邻块撞倒；在已经搭好的塔旁边，
一次横移就能把塔扫掉。

现场另一处隐患：原来每次 `set_pose` 之后固定 `sleep(1)`，不管距离多远。
如果下一条指令在运动未完成时到达，控制器会**重新规划**，直接把弧线抄近路
抄到砖堆里 —— 表现就是"上升途中撞到别的物体"。

本模块把动作拆成"小步 + 停顿"：
  * 抬升：同一点分 3 段竖直上升，每段带极小的后撤偏置，让弧线漂移朝"离开
    砖堆"的方向；
  * 放置：先到目标的外侧上方 -> 平移到正上方 -> 4 段下降 -> 松泵 -> 斜向后撤
    （绝不贴着塔面竖直起）；
  * 每一步下发前用 IK 模型自检，够不到就不下发（固件遇到不可达位姿会
    **静默不动**，命令却照常回显，看起来像"已到位"）。

所有数值都可以在 config/arm_stack.yaml 里改，不用改代码。
"""

import math
import os
import time

# --------------------------------------------------------------------------
# uArm Swift Pro IK 模型（与 swiftpro_rviz_node.cpp / match_logic.py 同一套）
# 用于"下发前自检"，避免固件静默忽略不可达位姿（E22 unreachable）。
# --------------------------------------------------------------------------
MATH_TRANS = 57.2958
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_UPPER_LOWER = MATH_UPPER_ARM / MATH_LOWER_ARM
MATH_Z_OFFSET = 74.55
MATH_X_OFFSET = 56.55

# 固件/机械限位
LIM_T1_MIN, LIM_T1_MAX = 0.0, 135.6      # 下臂
LIM_T2_MIN, LIM_T2_MAX = 0.0, 100.7      # 上臂
LIM_SUM_MIN, LIM_SUM_MAX = 10.0, 151.0   # 两关节之和


def ik_reachable(x, y, z):
    """机械臂能否到达 (x, y, z)（mm）。与固件的判定同源。"""
    try:
        z = z + MATH_Z_OFFSET
        z_in = (z - MATH_L1) / MATH_LOWER_ARM
        if x < 0.1:
            x = 0.1
        if y == 0:
            rot = 90.0
        elif y < 0:
            rot = -math.atan(x / y) * MATH_TRANS
        else:
            rot = 180.0 - math.atan(x / y) * MATH_TRANS

        sin_rot = math.sin(rot / MATH_TRANS)
        if sin_rot == 0:
            return False

        x_in = (x / sin_rot - MATH_L2 - MATH_X_OFFSET) / MATH_LOWER_ARM
        if x_in == 0:
            return False

        sqrt_zx = math.sqrt(z_in * z_in + x_in * x_in)
        if sqrt_zx == 0:
            return False

        upper = MATH_UPPER_LOWER
        right_all = (sqrt_zx * sqrt_zx + upper * upper - 1) / (2 * upper * sqrt_zx)
        left_all = (sqrt_zx * sqrt_zx + 1 - upper * upper) / (2 * sqrt_zx)
        if abs(right_all) > 1 or abs(left_all) > 1:
            return False

        phi = math.atan2(z_in, x_in) * MATH_TRANS
        angle_right = math.acos(right_all) * MATH_TRANS - phi
        angle_left = math.acos(left_all) * MATH_TRANS + phi

        return (LIM_T1_MIN <= angle_left <= LIM_T1_MAX and
                LIM_T2_MIN <= angle_right <= LIM_T2_MAX and
                LIM_SUM_MIN <= (angle_left + angle_right) <= LIM_SUM_MAX)
    except Exception:
        return False


def z_max(x, y, step=2.0, lo=-100.0, hi=260.0):
    """该 (x, y) 上 IK 可达的最大 z；不可达返回 None。"""
    best = None
    v = lo
    while v <= hi:
        if ik_reachable(x, y, v):
            best = v
        v += step
    return best


def cap_z(x, y, want, step=1.0, search=120.0):
    """从 want 往下找最近的"真的可达"的 z。

    IK 可达域在少数位置是不连续的（z_max 下面还有空洞），所以不能只用
    z_max 判断，必须实际往下探。
    """
    v = float(want)
    floor = v - float(search)
    while v >= floor:
        if ik_reachable(x, y, v):
            return round(v, 2)
        v -= step
    return None


# --------------------------------------------------------------------------
# 配置
# --------------------------------------------------------------------------
DEFAULTS = {
    "enabled": True,

    # 每段竖直小步的高度（mm）。越小越稳、越慢。
    "step_mm": 20.0,
    # 每段之间的停顿（s）。够固件把这一步走完，避免下一条指令插进来改轨迹。
    "step_dwell": 0.45,
    # 每段抬升时朝 -x（机械臂底座方向）的偏置，用来把弧线漂移压向"离开砖堆"
    "lift_bias_x": -4.0,
    "lift_bias_y": 0.0,

    # 抓取后要抬到"接触高度 + 这个值"（mm）。必须 > 砖高 + 20，否则横移会蹭到邻块。
    "lift_after_grasp": 120.0,
    "lift_after_grasp_cake": 90.0,

    # 放置：目标外侧偏移（mm，朝 -x = 靠近车体 = 塔的外侧）
    "approach_dx": -45.0,
    "approach_dy": 0.0,
    # 放置前先到这个高度（相对目标 z，mm）
    "approach_up": 45.0,
    # 下降分段数
    "descend_steps": 4,
    # 松泵后停留（s），让真空完全泄掉再撤
    "release_dwell": 0.5,
    # 松泵后先斜向撤到的相对高度（mm）
    "retract_up": 25.0,

    # 下发前 IK 自检：目标 ± 这个余量都必须可达
    "ik_margin": 0.0,
    # 超范围时是否只警告不下发（True）；False = 仍然下发（旧行为）
    "ik_guard": True,

    # 每一层放置的腕部 z（mm）。留空 = 用 contact_z + layer*brick_h 推算。
    # 实测后可以填死，例如： [ -55, 45, 145 ]
    "layer_z": [],
    # 第一层接触高度（腕部坐标）
    "contact_z": -55.0,
    # 金砖高度：第二层 = contact_z + brick_h，第三层 = contact_z + 2*brick_h
    # ★ 必须实测！这决定第三层到底够不够得着（腕部 z 上限约 174mm）
    "brick_h": 100.0,
    # 第三层够不到时，是否退化成"放到地面"（保 1 分，别让塔塌）
    "floor_fallback": True,
    # 退化落点（相对塔位的 y 偏移，mm）
    "fallback_dy": -110.0,

    # set_pose 的等待模型：每毫米耗时（s/mm），下限/上限（s）
    "sec_per_mm": 0.0045,
    "wait_min": 0.55,
    "wait_max": 6.0,
}


def load_config(path=None, log=None):
    """读 yaml（可选），缺失时返回默认值。纯函数，方便单测。"""
    cfg = dict(DEFAULTS)
    if not path or not os.path.exists(path):
        if log:
            log("arm_stack: 未找到 %s，使用内置默认参数" % path)
        return cfg
    try:
        import yaml
        with open(path, "r", encoding="utf8") as handle:
            data = yaml.safe_load(handle) or {}
        if isinstance(data, dict):
            for key, value in data.items():
                if key in cfg and value is not None:
                    cfg[key] = value
            cfg["layer_z"] = [float(v) for v in (data.get("layer_z") or [])]
        if log:
            log("arm_stack: 已加载 %s" % path)
    except Exception as exc:                                  # noqa: BLE001
        if log:
            log("arm_stack: 配置读取失败(%s)，使用默认参数" % exc)
    return cfg


# --------------------------------------------------------------------------
# 运动
# --------------------------------------------------------------------------
class StackMotion(object):
    """把 SwiftProInterface 包装成"小步、可校验、不抄近路"的运动序列。"""

    def __init__(self, interface, cfg=None, log=None):
        self.iface = interface
        self.cfg = dict(DEFAULTS if cfg is None else cfg)
        self.log = log or (lambda msg: None)
        self.enabled = bool(self.cfg.get("enabled", True))
        # 没有真实位置反馈（uArm 是开环步进），只能按时间等；这里记一下累计耗时，
        # 方便赛后核算"堆叠到底慢了多少秒"。
        self.spent = 0.0

    # ---------- 基础 ----------
    def _wait(self, start, target):
        """按距离估算等待时间，并把时间记到 spent。"""
        dist = math.sqrt(sum((float(b) - float(a)) ** 2 for a, b in zip(start, target)))
        secs = dist * float(self.cfg["sec_per_mm"])
        secs = max(float(self.cfg["wait_min"]), min(float(self.cfg["wait_max"]), secs))
        time.sleep(secs)
        self.spent += secs

    def reachable(self, x, y, z):
        """IK 自检：目标本身和 ±margin 的球都要可达。"""
        margin = float(self.cfg.get("ik_margin", 0.0))
        probes = [(x, y, z)]
        if margin > 0:
            probes += [(x + margin, y, z), (x - margin, y, z),
                       (x, y + margin, z), (x, y - margin, z),
                       (x, y, z + margin), (x, y, z - margin)]
        return all(ik_reachable(px, py, pz) for px, py, pz in probes)

    def move(self, x, y, z, guard=True):
        """下发一个绝对位姿并等待。不可达时按 ik_guard 决定是否跳过。"""
        x, y, z = float(x), float(y), float(z)
        if guard and self.cfg.get("ik_guard", True) and not self.reachable(x, y, z):
            self.log("arm_stack: 拒发不可达位姿 (%.1f, %.1f, %.1f) —— IK 判否" % (x, y, z))
            return False
        prev = getattr(self, "_last", None)
        self.iface.set_pose_nowait(x, y, z)
        if prev is None:
            time.sleep(float(self.cfg["wait_min"]))
        else:
            self._wait(prev, (x, y, z))
        self._last = (x, y, z)
        return True

    # ---------- 组合动作 ----------
    def lift(self, x, y, z_from, dz, stages=None):
        """同一点竖直分段抬升（带朝外的微小偏置）。

        对应现场已验证的做法（tools/c_calibration/pick_current_contact.py：
        62.43 -> 82.0 -> 105.0 两段抬升，每段确认到位）。单段抬 100mm 的弧线
        漂移在密集阵列里一定会撞到邻块。
        """
        if not self.enabled:
            self.iface.set_pose(x, y, z_from + dz)
            return True

        step_mm = max(5.0, float(self.cfg["step_mm"]))
        # 抬到臂展上限就停：多抬的指令会被固件静默丢弃，反而让人以为"抬到位了"
        capped = cap_z(x, y, z_from + dz)
        if capped is None:
            self.log("arm_stack: 抬升目标不可达，放弃 (x=%.0f, y=%.0f)" % (x, y))
            return False
        if capped < z_from + dz - 0.5:
            self.log("arm_stack: 抬升 %.0fmm 超过可达上限，压到 %.0fmm"
                     % (dz, capped - z_from))
            dz = capped - z_from
        stages = int(stages or max(2, math.ceil(abs(dz) / step_mm)))
        bias_x = float(self.cfg.get("lift_bias_x", 0.0))
        bias_y = float(self.cfg.get("lift_bias_y", 0.0))

        self._last = (x, y, z_from)
        for i in range(1, stages + 1):
            frac = float(i) / stages
            tx = x + bias_x * frac
            ty = y + bias_y * frac
            tz = z_from + dz * frac
            if not self.move(tx, ty, tz):
                return False
            time.sleep(float(self.cfg["step_dwell"]))
        return True

    def place(self, x, y, z, up=None, descend_steps=None):
        """在 (x, y, z) 放件：外侧上方 -> 正上方 -> 分段下降 -> 松泵 -> 斜向外撤。"""
        if not self.enabled:
            self.iface.set_pose(x, y, z)
            return True

        dx = float(self.cfg.get("approach_dx", 0.0))
        dy = float(self.cfg.get("approach_dy", 0.0))
        up = float(up if up is not None else self.cfg["approach_up"])
        steps = int(descend_steps or self.cfg["descend_steps"])
        retract_up = float(self.cfg.get("retract_up", 25.0))

        # 三层塔的塔顶已经贴着臂展上限：接近高度必须按 IK 上限压下来，
        # 否则姿态不可达，固件静默不动（看起来"抬到位了"其实没动）。
        if not ik_reachable(x, y, z):
            self.log("arm_stack: 放置点 (%.0f, %.0f, %.0f) 本身不可达，放弃本次放置"
                     % (x, y, z))
            return False
        head = cap_z(x + dx, y + dy, z + up)
        if head is None:
            self.log("arm_stack: 目标外侧 (%.0f, %.0f) 附近没有可达高度，放弃"
                     % (x + dx, y + dy))
            return False
        if head < z + up:
            self.log("arm_stack: z=%.0f 接近上限，接近高度 %.0f->%.0fmm"
                     % (z, up, max(2.0, head - z)))
        up = max(2.0, head - z)
        retract_up = min(retract_up, up)

        # 1) 外侧上方
        if not self.move(x + dx, y + dy, z + up):
            return False
        time.sleep(float(self.cfg["step_dwell"]))
        # 2) 正上方
        if not self.move(x, y, z + up):
            return False
        time.sleep(float(self.cfg["step_dwell"]))
        # 3) 分段下降
        for i in range(1, steps + 1):
            frac = float(i) / steps
            if not self.move(x, y, z + up * (1.0 - frac)):
                return False
            time.sleep(float(self.cfg["step_dwell"]))
        # 4) 松泵 + 停留
        self.iface.set_pump(False)
        time.sleep(float(self.cfg["release_dwell"]))
        # 5) 斜向撤离：先上再外，绝不贴塔面横切
        self.move(x + dx, y + dy, z + retract_up)
        return True

    def retract_to(self, x, y, z_target, up=None):
        """从当前位置先抬到 z_target，再平移到 (x, y)。用于回 ready 位。"""
        up = float(up if up is not None else self.cfg.get("approach_up", 45.0))
        cur = getattr(self, "_last", (x, y, z_target))
        return (self.move(cur[0], cur[1], z_target + up) and
                self.move(x, y, z_target))


def layer_z_for(cfg, layer):
    """第 layer 层（0 起）放置时的腕部 z。

    优先级：cfg['layer_z'] 表 -> contact_z + layer * brick_h。
    返回 (z, 来源说明)，方便日志里看清用的是哪来的数。
    """
    table = list(cfg.get("layer_z") or [])
    if len(table) > layer:
        return float(table[layer]), "layer_z 表"
    contact = float(cfg.get("contact_z", -55.0))
    brick_h = float(cfg.get("brick_h", 100.0))
    return contact + layer * brick_h, "contact_z + %d*%.0f" % (layer, brick_h)


def describe():
    """打印当前默认参数，方便现场核对。"""
    for key in sorted(DEFAULTS):
        print("%-24s %s" % (key, DEFAULTS[key]))


if __name__ == "__main__":
    describe()
