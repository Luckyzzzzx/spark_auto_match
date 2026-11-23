#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单块金砖 G/M 配对标定（按你的节奏：一次一块砖，两个键）。

流程
----
1) 场上只放一块金砖，画面里会自动框住它（黄框 + 中心点 + 顶面中心十字）
2) 按 G  —— 记录这块砖现在的像素 (u, v)，程序把这一帧的检测冻结下来
3) 你拨机械臂（W/S 前后、A/D 左右、R/F 上下），让橙色吸盘压到砖顶面中心
4) 按 M  —— 记录当前机械臂 (x, y, z)，和刚才的像素配成一组
5) 换下一块砖（或把这块挪到别处），重复 1~4，至少 4 组
6) 按 F  —— 拟合 x = a*v + b, y = c*u + d，打印每组残差
7) 按 K  —— 残差合格才写入 ~/thefile.txt（自动备份）

画面上的辅助信息
---------------
* 红十字 = 砖的顶面中心（吸盘该压的位置）
* 绿圈   = 当前识别到的橙色吸盘中心
* 左上角 = 吸盘相对红十字的偏差（px），越小越准（<10px 就很好）
* 建议所有组都在同一个 z（吸盘贴住砖顶），z 相差超过 10mm 会警告

按键（焦点要在图像窗口里，先用鼠标点一下）
    W/S  y ±      A/D  x ∓      R/F  z ±
    + / -  步长循环 2 / 5 / 10 / 20 mm
    G  记砖像素    M  记机械臂坐标并配对
    H  回 ready 位
    P  打印已配对    F  拟合    K  写文件
    X  清空重来    Q  退出（点也存到 /tmp/teach_pairs.txt）

用法
----
    cd ~/spark_noetic/src/3rd_app/auto_match_25/auto_match
    DISPLAY=:0 python3 scripts/teach_pair.py
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

TOP_FRAC = 0.25
STEPS = (2.0, 5.0, 10.0, 20.0)
LIMITS = {'x': (0.0, 320.0), 'y': (-250.0, 250.0), 'z': (-100.0, 200.0)}
OUT = os.path.expanduser("~/thefile.txt")
LOG = "/tmp/teach_pairs.txt"
READY = [50.0, 180.0, 160.0]


class PairTeach(object):
    def __init__(self):
        self.pos = list(READY)
        self.step_idx = 1
        self.det = None          # 当前帧检测到的砖 (id,u,v,h)
        self.frozen = None       # 按 G 冻结的 (id,u,v,h)
        self.pairs = []          # [((x,y,z),(u,v,h))]
        self.fit_res = None
        self.msg = "ready"
        self.pub = rospy.Publisher("position_write_topic", position, queue_size=1)
        self.en_pub = rospy.Publisher("swiftpro_status_topic", status, queue_size=1)
        rospy.Subscriber("/objects", Detection2DArray, self.cb)
        self.bridge = CvBridge()
        rospy.sleep(1.0)
        self.en_pub.publish(status(1))
        self.send()

    # ---------- 基础 ----------
    def cb(self, msg):
        best = None
        for o in msg.detections:
            if not o.results or not (1 <= o.results[0].id <= 5):
                continue
            area = float(o.bbox.size_x) * float(o.bbox.size_y)
            if best is None or area > best[4]:
                best = (int(o.results[0].id), float(o.bbox.center.x),
                        float(o.bbox.center.y), float(o.bbox.size_y), area)
        self.det = best[:4] if best else None

    @property
    def step(self):
        return STEPS[self.step_idx]

    def send(self):
        for i, axis in enumerate(('x', 'y', 'z')):
            lo, hi = LIMITS[axis]
            if not (lo <= self.pos[i] <= hi):
                self.msg = "%s out of range (%.0f,%.0f,%.0f)" % (axis, *self.pos)
                return
        self.pub.publish(position(x=self.pos[0], y=self.pos[1], z=self.pos[2]))
        self.msg = "arm=(%.1f, %.1f, %.1f)  step=%.0fmm" % (*self.pos, self.step)

    def frame(self):
        return self.bridge.imgmsg_to_cv2(
            rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5), "bgr8")

    @staticmethod
    def cup_pixel(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 140, 150), (25, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        if n <= 1:
            return None
        i = 1 + int(stats[1:, cv2.CC_STAT_AREA].argmax())
        if stats[i, cv2.CC_STAT_AREA] < 300:
            return None
        return (float(cents[i][0]), float(cents[i][1]))

    # ---------- 显示 ----------
    def draw(self, img):
        cup = self.cup_pixel(img)
        c = self.frozen or self.det
        if c:
            bid, u, v, h = c
            tu, tv = int(u), int(v - TOP_FRAC * h)
            color = (0, 255, 255) if self.frozen else (0, 200, 0)
            cv2.rectangle(img, (int(u - h / 2), int(v - h / 2)),
                          (int(u + h / 2), int(v + h / 2)), color, 1)
            cv2.circle(img, (int(u), int(v)), 3, color, -1)
            cv2.drawMarker(img, (tu, tv), color, cv2.MARKER_CROSS, 26, 2)
            cv2.putText(img, "brick %s" % ("FROZEN" if self.frozen else ""),
                        (int(u) - 40, int(v - h / 2) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if cup:
            cv2.circle(img, (int(cup[0]), int(cup[1])), 10, (0, 255, 0), 2)
            if c:
                tu, tv = c[1], c[2] - TOP_FRAC * c[3]
                cv2.line(img, (int(cup[0]), int(cup[1])), (int(tu), int(tv)), (0, 255, 0), 1)
                cv2.putText(img, "off %.0fpx" % np.hypot(cup[0] - tu, cup[1] - tv),
                            (int(cup[0]) + 12, int(cup[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(img, self.msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(img, "pairs=%d  frozen=%s" % (
            len(self.pairs), "yes" if self.frozen else "no"),
            (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(img, "W/S y  A/D x  R/F z  +/- step  G brick  M arm  H ready  P print  F fit  K write  X clear  Q quit",
                    (8, 472), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1)
        return img

    # ---------- 拟合 ----------
    def fit(self):
        if len(self.pairs) < 3:
            self.msg = "only %d pairs, need >=3" % len(self.pairs)
            return None
        V = np.array([p[1][1] for p in self.pairs], float)
        U = np.array([p[1][0] for p in self.pairs], float)
        X = np.array([p[0][0] for p in self.pairs], float)
        Y = np.array([p[0][1] for p in self.pairs], float)
        Zs = np.array([p[0][2] for p in self.pairs], float)
        a, b = np.polyfit(V, X, 1)
        c, d = np.polyfit(U, Y, 1)
        worst = 0.0
        print("\n=== 拟合（%d 组）===" % len(self.pairs), flush=True)
        print("x = %+.6f * v + %+.6f" % (a, b), flush=True)
        print("y = %+.6f * u + %+.6f" % (c, d), flush=True)
        for (ax, ay, az), (u, v, h) in self.pairs:
            ex, ey = ax - (a * v + b), ay - (c * u + d)
            worst = max(worst, abs(ex), abs(ey))
            print("  arm=(%6.1f,%7.1f,%5.1f) px=(%3.0f,%3.0f) -> 预测(%6.1f,%7.1f) 误差(%+6.1f,%+6.1f)"
                  % (ax, ay, az, u, v, a * v + b, c * u + d, ex, ey), flush=True)
        print("最大残差 %.1fmm | v 覆盖 %.0f~%.0f | u 覆盖 %.0f~%.0f | z 跨度 %.0fmm"
              % (worst, V.min(), V.max(), U.min(), U.max(), Zs.max() - Zs.min()), flush=True)
        if Zs.max() - Zs.min() > 10.0:
            print("!! 各组的 z 相差 %.0fmm：像素->机械臂的映射是按平面算的，"
                  "z 不一致会引入误差" % (Zs.max() - Zs.min()), flush=True)
        self.msg = "fit worst %.1fmm -> press K to write" % worst
        self.fit_res = (a, b, c, d, worst)
        return self.fit_res

    # ---------- 主循环 ----------
    def run(self):
        win = "teach_pair  (G=brick  M=arm)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1100, 900)
        print("放一块金砖 -> 按 G 记像素 -> 拨机械臂对准 -> 按 M 记坐标。", flush=True)
        while not rospy.is_shutdown():
            try:
                cv2.imshow(win, self.draw(self.frame()))
            except Exception as exc:                       # noqa: BLE001
                print("取图失败:", exc, flush=True)
            k = cv2.waitKey(30) & 0xFF
            if k == 255:
                continue
            ch = chr(k).lower()
            if ch in "wsadrf":
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
                else:
                    self.pos[2] -= self.step
                self.send()
            elif ch == 'g':
                if self.det is None:
                    self.msg = "no brick detected"
                else:
                    self.frozen = self.det
                    self.msg = "brick px=(%.0f,%.0f) frozen" % (self.frozen[1], self.frozen[2])
                    print("G: 砖 id=%d px=(%.0f,%.0f) h=%.0f"
                          % (self.frozen[0], self.frozen[1], self.frozen[2], self.frozen[3]),
                          flush=True)
            elif ch == 'm':
                if self.frozen is None:
                    self.msg = "press G first"
                else:
                    self.pairs.append((tuple(self.pos), self.frozen[1:]))
                    print("M: 配对 #%d arm=(%.1f,%.1f,%.1f) <-> px=(%.0f,%.0f)"
                          % (len(self.pairs), *self.pos, self.frozen[1], self.frozen[2]),
                          flush=True)
                    self.msg = "paired #%d (G frozen cleared)" % len(self.pairs)
                    self.frozen = None
                    self.fit_res = None
            elif ch == 'h':
                self.pos = list(READY)
                self.send()
            elif ch in "+-":
                self.step_idx = (self.step_idx + (1 if ch == '+' else -1)) % len(STEPS)
                self.msg = "step -> %.0fmm" % self.step
            elif ch == 'p':
                for i, ((ax, ay, az), (u, v, h)) in enumerate(self.pairs, 1):
                    print("  #%d arm=(%.1f,%.1f,%.1f) px=(%.0f,%.0f) h=%.0f"
                          % (i, ax, ay, az, u, v, h), flush=True)
                self.msg = "printed %d pairs" % len(self.pairs)
            elif ch == 'f':
                self.fit()
            elif ch == 'k':
                if not self.fit_res:
                    self.msg = "press F first"
                elif self.fit_res[4] > 20.0:
                    self.msg = "worst %.1fmm too big, NOT written" % self.fit_res[4]
                else:
                    line = "%.6f %.6f %.6f %.6f\n" % self.fit_res[:4]
                    if os.path.exists(OUT):
                        shutil.copyfile(OUT, OUT + ".bak_" + time.strftime("%Y%m%d_%H%M"))
                    with open(OUT, "w") as f:
                        f.write(line)
                    self.msg = "WRITTEN thefile.txt (worst %.1fmm)" % self.fit_res[4]
                    print("写入 %s: %s" % (OUT, line.strip()), flush=True)
            elif ch == 'x':
                self.pairs = []
                self.fit_res = None
                self.frozen = None
                self.msg = "cleared"
            elif ch == 'q':
                break
        cv2.destroyAllWindows()
        with open(LOG, "w") as f:
            for pose, cube in self.pairs:
                f.write("%r %r\n" % (pose, cube))
        print("退出：%d 组已存 %s" % (len(self.pairs), LOG), flush=True)


if __name__ == "__main__":
    rospy.init_node("teach_pair", anonymous=True)
    try:
        PairTeach().run()
    except KeyboardInterrupt:
        pass
