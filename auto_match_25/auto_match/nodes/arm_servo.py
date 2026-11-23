#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""吸盘视觉对准（最后的闭环微调）。

为什么需要它
------------
thefile/thefile2 是"像素 -> 机械臂"的**线性**映射，但相机是斜装的：
像素 v 和臂 x 在俯视几何里其实是射影关系（非线性）。标定点挤在一条窄带里时，
带内误差只有几毫米，一离开这条带就会偏几十毫米。所以映射只能用来把吸盘
送到目标附近（±60mm 足够），最后由这里闭环收敛到吸盘正对砖顶面中心。

约定
----
"对准"= 橙色吸盘的像素中心 对准 砖的**顶面中心**（框中心往上 0.25 个框高）。
和 teach_pad 的 M 步同一个标准（吸盘压住砖顶面中心）。
"""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CupServo(object):
    def __init__(self, topic="/camera/color/image_raw"):
        self.topic = topic
        self.bridge = CvBridge()

    def frame(self, timeout=6.0):
        msg = rospy.wait_for_message(self.topic, Image, timeout=timeout)
        return self.bridge.imgmsg_to_cv2(msg, "bgr8")

    @staticmethod
    def cup_pixel(img, min_area=300):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 140, 150), (25, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        if n <= 1:
            return None
        i = 1 + int(stats[1:, cv2.CC_STAT_AREA].argmax())
        if stats[i, cv2.CC_STAT_AREA] < min_area:
            return None
        return (float(cents[i][0]), float(cents[i][1]))

    def align(self, iface, target_uv, x, y, z, kb, iters=4, gain=0.7,
              tol=5.0, log=None):
        """在 z 平面把吸盘对准 target_uv；返回 (x, y, ok)。

        kb = (kb_x[0], kb_y[0]) = 映射里 v->x、u->y 的斜率（负值）：
             x = kb[0]*v + b,  y = kb[1]*u + d
        于是像素误差 -> 臂修正：dx = -kb[0]*dv, dy = -kb[1]*du
        """
        log = log or (lambda *_: None)
        tu, tv = target_uv
        for i in range(iters):
            img = self.frame()
            cp = self.cup_pixel(img)
            if cp is None:
                log("吸盘不可见，放弃对准")
                return x, y, False
            du, dv = cp[0] - tu, cp[1] - tv
            log("  对准#%d cup=(%.0f,%.0f) 误差=(%+.0f,%+.0f)px"
                % (i + 1, cp[0], cp[1], du, dv))
            if abs(du) <= tol and abs(dv) <= tol:
                return x, y, True
            x += -float(kb[0]) * dv * gain
            y += -float(kb[1]) * du * gain
            iface.set_pose(x, y, z)
        return x, y, False
