#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三层堆叠验证：直接走 runtask.ArmAction 的真实流程。

不启动导航、不做额外视觉标注，只把比赛里那三个动作单独跑三轮：
    grasp()            -> 用 thefile.txt 手眼标定 + arm_stack 分段抬升
    drop_layer(0/1/2)  -> 固定点/塔顶检测 + arm_stack 外侧接近、分段下降、斜撤

每步存图 /tmp/stack_verify_L*.jpg，便于事后核对塔有没有立住。

用法（机器人上，bringup + swiftpro + vision 已启动）:
    python3 scripts/stack_verify.py             # 三层全跑
    python3 scripts/stack_verify.py --layers 2  # 只跑两层
    python3 scripts/stack_verify.py --dry-run   # 只打印计划，不动机械臂
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
NODES = os.path.abspath(os.path.join(HERE, "..", "nodes"))
for _p in (NODES, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rospy                                     # noqa: E402
from cv_bridge import CvBridge                   # noqa: E402
from sensor_msgs.msg import Image                # noqa: E402

import cv2                                       # noqa: E402


def snapshot(path):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(
        rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10), "bgr8")
    cv2.imwrite(path, img)
    rospy.loginfo("图片: %s", path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layers", type=int, default=3, help="跑几层（1~3）")
    parser.add_argument("--zone", default="Collection_A", help="放置区名（只用于记忆塔位）")
    parser.add_argument("--pause", type=float, default=2.0, help="每层之间停留秒数")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不动机械臂")
    args = parser.parse_args()

    rospy.init_node("stack_verify", anonymous=True)

    if args.dry_run:
        sys.path.insert(0, NODES)
        from arm_stack import load_config, layer_z_for, ik_reachable
        cfg_path = os.path.join(HERE, "..", "config", "arm_stack.yaml")
        cfg = load_config(cfg_path, log=print)
        print("塔位(第一层) 固定点 = (235, 0)")
        for layer in range(args.layers):
            z, src = layer_z_for(cfg, layer)
            ok = ik_reachable(235.0, 0.0, z)
            print("第 %d 层: z=%.0f  可达=%-5s  来源=%s" % (layer + 1, z, ok, src))
        print("（未动机械臂）")
        return 0

    import runtask

    arm = runtask.ArmAction()
    rospy.loginfo("=== 三层堆叠验证开始：%d 层，区=%s ===", args.layers, args.zone)
    done = 0
    for layer in range(args.layers):
        rospy.loginfo("---------- 第 %d 层 ----------", layer + 1)
        try:
            item = arm.grasp()
        except Exception as exc:                             # noqa: BLE001
            rospy.logerr("抓取异常，停下（保持当前吸持状态，需人工确认）: %s", exc)
            return 1
        if not item:
            rospy.logwarn("没抓到东西，停止")
            return 1
        rospy.loginfo("已抓到 id=%s，开始放第 %d 层", item, layer + 1)
        try:
            arm.drop_layer(layer, zone=args.zone)
        except Exception as exc:                             # noqa: BLE001
            rospy.logerr("放置异常，停下（机械臂保持原位）: %s", exc)
            return 1
        done += 1
        snapshot("/tmp/stack_verify_L%d.jpg" % (layer + 1))
        rospy.loginfo("第 %d 层完成（累计 %d 层）", layer + 1, done)
        rospy.sleep(args.pause)

    arm.arm_grasp_ready()
    rospy.loginfo("=== 结束：完成 %d 层 ===", done)
    return 0 if done == args.layers else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
