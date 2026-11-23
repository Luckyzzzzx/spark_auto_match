#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手眼标定（G/M 两步法，带十字准星的可视化版）。

流程和 teach_pad 一致：
  1) 画面里给每块砖编号，十字准星标出它的"顶面中心"
  2) 你在窗口里按键拨机械臂，让橙色吸盘正中压在十字上
  3) 按 M 记录一组：砖的框中心像素 + 当前机械臂 (x,y,z)
  4) 按 F 拟合并打印每组残差；合格后按 K 写入 ~/thefile.txt（自动备份）

按键（焦点要在图像窗口里，先用鼠标点一下窗口）：
  W/S  y ±      A/D  x ∓      R/F  z ±
  + / -  循环切换步长 5 / 10 / 2 mm
  1~9    选中第几块砖（会重新取它的像素）
  O      回 ready 位（换下一块砖前先按）
  M      记录一组      P  打印已记录
  F      拟合          K  写入 thefile.txt      Q  退出

建议至少 4 组，覆盖近/远/左/右（v 从 200 到 400 都要取到），
否则线性拟合只在中间那条窄带里准。

用法：
  cd ~/spark_noetic/src/3rd_app/auto_match_25/auto_match
  DISPLAY=:0 python3 scripts/teach_pad2.py
"""
import os
import shutil
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from swiftpro.msg import position, status
from vision_msgs.msg import Detection2DArray

# 吸盘视觉对准（自动微调用），nodes/ 目录
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                "..", "nodes")))
from arm_servo import CupServo                      # noqa: E402

TOP_FRAC = 0.25
STEPS = (5.0, 10.0, 2.0)
LIMITS = {'x': (0.0, 320.0), 'y': (-250.0, 250.0), 'z': (-100.0, 200.0)}
OUT = os.path.expanduser("~/thefile.txt")
LOG = "/tmp/teach_points2.txt"


class _Iface(object):
    """给 CupServo 用的最小接口：改位姿就下发。"""

    def __init__(self, outer):
        self.o = outer

    def set_pose(self, x, y, z):
        self.o.pos = [float(x), float(y), float(z)]
        self.o.send()


class Teach(object):
    def __init__(self):
        self.pos = [50.0, 180.0, 160.0]      # ready：砖不被手臂挡住
        self.step_idx = 0
        self.cubes = []                      # [(id, u, v, h)]
        self.sel = None
        self.pairs = []                      # [((x,y,z), (u,v,h))]
        self.msg = "ready"
        self.servo = CupServo()
        self.pub = rospy.Publisher("position_write_topic", position, queue_size=1)
        self.status_pub = rospy.Publisher("swiftpro_status_topic", status, queue_size=1)
        rospy.Subscriber("/objects", Detection2DArray, self.cb)
        self.bridge = CvBridge()
        self.iface = _Iface(self)
        rospy.sleep(1.0)
        self.status_pub.publish(status(1))
        self.send()

    def cb(self, msg):
        self.cubes = [(int(o.results[0].id),
                       float(o.bbox.center.x), float(o.bbox.center.y),
                       float(o.bbox.size_y))
                      for o in msg.detections if o.results and 1 <= o.results[0].id <= 5]
        self.cubes.sort(key=lambda c: c[1])          # 从左到右编号

    @property
    def step(self):
        return STEPS[self.step_idx]

    def send(self):
        for i, axis in enumerate(('x', 'y', 'z')):
            lo, hi = LIMITS[axis]
            if not (lo <= self.pos[i] <= hi):
                self.msg = "%s limit! (%.0f,%.0f,%.0f)" % (axis, *self.pos)
                return
        self.pub.publish(position(x=self.pos[0], y=self.pos[1], z=self.pos[2]))
        self.msg = "arm=(%.1f, %.1f, %.1f)  step=%.0fmm" % (*self.pos, self.step)

    def frame(self):
        return self.bridge.imgmsg_to_cv2(
            rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5), "bgr8")

    def draw(self, img):
        for i, (bid, u, v, h) in enumerate(self.cubes, 1):
            tu, tv = int(u), int(v - TOP_FRAC * h)
            sel = self.sel is not None and abs(self.sel[1] - u) < 1e-6
            color = (0, 255, 255) if sel else (0, 200, 0)
            cv2.rectangle(img, (int(u - 50), int(v - 50)), (int(u + 50), int(v + 50)), color, 1)
            cv2.putText(img, "%d" % i, (int(u) - 55, int(v) - 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.drawMarker(img, (tu, tv), color, cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img, self.msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(img, "pairs=%d  sel=%s" % (
            len(self.pairs), "none" if self.sel is None else "id%d" % self.sel[0]),
            (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(img, "W/S y  A/D x  R/F z  +/- step  1-9 pick  O ready  G auto-align  M rec  F fit  K write  Q quit",
                    (8, 472), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1)
        return img

    def fit(self):
        if len(self.pairs) < 3:
            self.msg = "only %d pairs, need >=3" % len(self.pairs)
            return None
        V = np.array([p[1][1] for p in self.pairs], float)
        U = np.array([p[1][0] for p in self.pairs], float)
        X = np.array([p[0][0] for p in self.pairs], float)
        Y = np.array([p[0][1] for p in self.pairs], float)
        a, b = np.polyfit(V, X, 1)
        c, d = np.polyfit(U, Y, 1)
        worst = 0.0
        print("\n=== 拟合（%d 组）===" % len(self.pairs), flush=True)
        print("x = %+.6f * v + %+.6f" % (a, b), flush=True)
        print("y = %+.6f * u + %+.6f" % (c, d), flush=True)
        for (ax, ay, _az), (u, v, h) in self.pairs:
            ex, ey = ax - (a * v + b), ay - (c * u + d)
            worst = max(worst, abs(ex), abs(ey))
            print("  arm=(%6.1f,%6.1f) u=%3.0f v=%3.0f -> (%6.1f,%6.1f) err(%+5.1f,%+5.1f)"
                  % (ax, ay, u, v, a * v + b, c * u + d, ex, ey), flush=True)
        print("最大残差 %.1fmm   v 覆盖 %.0f~%.0f   u 覆盖 %.0f~%.0f"
              % (worst, V.min(), V.max(), U.min(), U.max()), flush=True)
        self.msg = "fit ok, worst %.1fmm  -> press K to write" % worst
        return (a, b, c, d, worst)

    def run(self):
        win = "teach_pad2  (W/S y  A/D x  R/F z)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1100, 900)
        print("开始：先在窗口里按 1~9 选砖，拨手臂把橙色吸盘压在红十字上，按 M 记录。",
              flush=True)
        result = None
        while not rospy.is_shutdown():
            try:
                cv2.imshow(win, self.draw(self.frame()))
            except Exception as exc:                       # noqa: BLE001
                print("取图失败:", exc, flush=True)
            k = cv2.waitKey(30) & 0xFF
            if k == 255:
                continue
            ch = chr(k).lower()
            moved = True
            if ch == 'w':
                self.pos[1] += self.step
            elif ch == 's':
                self.pos[1] -= self.step
            elif ch == 'a':
                self.pos[0] -= self.step
            elif ch == 'd':
                self.pos[0] += self.step
            elif ch == 'r':
                self.pos[2] += self.step
            elif ch == 'f':
                self.pos[2] -= self.step
            else:
                moved = False
            if moved:
                self.send()
            elif ch == 'o':
                self.pos = [50.0, 180.0, 160.0]
                self.send()
            elif ch in "+-":
                self.step_idx = (self.step_idx + (1 if ch == '+' else -1)) % len(STEPS)
                self.msg = "step -> %.0fmm" % self.step
            elif ch.isdigit() and 1 <= int(ch) <= len(self.cubes):
                self.sel = self.cubes[int(ch) - 1]
                self.msg = "picked id=%d px=(%.0f,%.0f) top v=%.0f" % (
                    self.sel[0], self.sel[1], self.sel[2],
                    self.sel[2] - TOP_FRAC * self.sel[3])
            elif ch == 'g':
                if self.sel is None:
                    self.msg = "press 1-9 first"
                else:
                    tu = self.sel[1]
                    tv = self.sel[2] - TOP_FRAC * self.sel[3]
                    x, y, ok = self.servo.align(
                        self.iface, (tu, tv), self.pos[0], self.pos[1], self.pos[2],
                        (-1.0, -0.9), iters=6, tol=5.0, log=print)
                    self.pos = [x, y, self.pos[2]]
                    self.msg = ("auto-align OK -> press M" if ok
                                else "auto-align FAILED, adjust by hand")
            elif ch == 'm':
                if self.sel is None:
                    self.msg = "press 1-9 to pick a brick first"
                else:
                    tu = self.sel[1]
                    tv = self.sel[2] - TOP_FRAC * self.sel[3]
                    cp = self.servo.cup_pixel(self.frame())
                    if cp is None:
                        self.msg = "cup not visible, bring it into view"
                    else:
                        off = ((cp[0] - tu) ** 2 + (cp[1] - tv) ** 2) ** 0.5
                        if off > 15.0:
                            self.msg = "NOT aligned (%.0fpx) - refused" % off
                            print("拒绝记录：吸盘离目标 %.0fpx（>15），先按 G 自动对准" % off,
                                  flush=True)
                        else:
                            self.pairs.append((tuple(self.pos), self.sel[1:]))
                            self.msg = "recorded pair #%d (off %.0fpx)" % (len(self.pairs), off)
                            print("记录 #%d arm=(%.1f,%.1f,%.1f) 砖 px=(%.0f,%.0f) h=%.0f 偏差%.0fpx"
                                  % (len(self.pairs), *self.pos, *self.sel[1:], off),
                                  flush=True)
            elif ch == 'p':
                for i, ((ax, ay, az), (u, v, h)) in enumerate(self.pairs, 1):
                    print("  #%d arm=(%.1f,%.1f,%.1f) px=(%.0f,%.0f) h=%.0f"
                          % (i, ax, ay, az, u, v, h), flush=True)
                self.msg = "printed %d pairs" % len(self.pairs)
            elif ch == 'f':
                result = self.fit()
            elif ch == 'k':
                if not result:
                    self.msg = "press F to fit first"
                elif result[4] > 20.0:
                    self.msg = "worst %.1fmm too big, NOT written" % result[4]
                else:
                    line = "%.6f %.6f %.6f %.6f\n" % result[:4]
                    if os.path.exists(OUT):
                        shutil.copyfile(OUT, OUT + ".bak_" + time.strftime("%Y%m%d_%H%M"))
                    with open(OUT, "w") as f:
                        f.write(line)
                    self.msg = "WRITTEN thefile.txt (worst %.1fmm)" % result[4]
                    print("写入 %s: %s" % (OUT, line.strip()), flush=True)
            elif ch == 'q':
                break
        cv2.destroyAllWindows()
        with open(LOG, "w") as f:
            for pose, cube in self.pairs:
                f.write("%r %r\n" % (pose, cube))
        print("退出，%d 组已存 %s" % (len(self.pairs), LOG), flush=True)


if __name__ == "__main__":
    rospy.init_node("teach_pad2", anonymous=True)
    try:
        Teach().run()
    except KeyboardInterrupt:
        pass
