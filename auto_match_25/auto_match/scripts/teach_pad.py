#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两步示教标定：解决"机械臂挡住金砖时识别不到"的问题。

流程：
  1) 金砖放在相机可见处（机械臂别挡），按 G 记录当前金砖像素
  2) 机械臂移到金砖正上方（吸盘对准），按 M 记录机械臂坐标
  配对保存：[机械臂(x,y,z), 金砖(id,px,py)]

按键：
  W/S 前后  A/D 左右  R/F 上下
  G 记金砖像素    M 记机械臂坐标（与上一块金砖配对）
  P 打印          Q 保存退出
"""
import select
import sys
import termios
import tty

import rospy
from swiftpro.msg import position, status
from vision_msgs.msg import Detection2DArray

STEP = 5.0
LIMITS = {'x': (0.0, 320.0), 'y': (-250.0, 250.0), 'z': (-100.0, 200.0)}


class Teach(object):
    def __init__(self):
        self.pos = [130.0, 0.0, 35.0]
        self.objs = []
        self.pending_cube = None
        self.points = []
        self.pub = rospy.Publisher('position_write_topic', position, queue_size=1)
        self.status_pub = rospy.Publisher('swiftpro_status_topic', status, queue_size=1)
        rospy.Subscriber('/objects', Detection2DArray, self.cb)
        rospy.sleep(1.0)
        self.status_pub.publish(status(1))
        self.send(force=True)
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def cb(self, msg):
        self.objs = [(int(o.results[0].id), round(o.bbox.center.x, 1), round(o.bbox.center.y, 1))
                     for o in msg.detections if o.results]

    def _limit_ok(self):
        for i, axis in enumerate(('x', 'y', 'z')):
            lo, hi = LIMITS[axis]
            if not (lo <= self.pos[i] <= hi):
                return False, axis
        return True, None

    def send(self, force=False):
        ok, axis = self._limit_ok()
        if not ok and not force:
            print("!! %s 超范围，已拒绝 pos=(%.1f,%.1f,%.1f)" % (axis, self.pos[0], self.pos[1], self.pos[2]), flush=True)
            return False
        self.pub.publish(position(x=self.pos[0], y=self.pos[1], z=self.pos[2]))
        return True

    def nearest_cube(self):
        # 只选金砖（id 1~5），排除 site 标记(10/20/30/40/50)和 base(99)
        cubes = [o for o in self.objs if 1 <= o[0] <= 5]
        if not cubes:
            return None
        return min(cubes, key=lambda o: (o[1] - 320) ** 2 + (o[2] - 240) ** 2)

    def key(self):
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            return sys.stdin.read(1)
        return None

    def run(self):
        tty.setcbreak(self.fd)
        print("两步示教：G 记金砖(金砖可见) -> M 记机械臂(吸盘对准) -> P 打印 -> Q 保存", flush=True)
        print("控制：W/S 前后  A/D 左右  R/F 上下", flush=True)
        while not rospy.is_shutdown():
            k = self.key()
            if not k:
                continue
            k = k.lower()
            moved = True
            if k == 'w':
                self.pos[1] += STEP
            elif k == 's':
                self.pos[1] -= STEP
            elif k == 'a':
                self.pos[0] -= STEP
            elif k == 'd':
                self.pos[0] += STEP
            elif k == 'r':
                self.pos[2] += STEP
            elif k == 'f':
                self.pos[2] -= STEP
            else:
                moved = False
            if moved:
                if self.send():
                    print("pos=(%.1f,%.1f,%.1f)" % tuple(self.pos), flush=True)
            elif k == 'g':
                c = self.nearest_cube()
                if c:
                    self.pending_cube = c
                    print("记录金砖: id=%d 像素=(%.0f,%.0f)  当前objs=%s" % (c[0], c[1], c[2], self.objs), flush=True)
                else:
                    print("!! /objects 没有金砖，无法记录（确保金砖在可见处）", flush=True)
            elif k == 'm':
                if self.pending_cube is None:
                    print("!! 还没记金砖，先按 G", flush=True)
                else:
                    self.points.append((list(self.pos), self.pending_cube))
                    print("配对记录 #%d: 机械臂=(%.1f,%.1f,%.1f) 金砖=%s" %
                          (len(self.points), self.pos[0], self.pos[1], self.pos[2], self.pending_cube), flush=True)
                    self.pending_cube = None
            elif k == 'p':
                print("--- 已配对 %d 点 ---" % len(self.points), flush=True)
                for i, (p, c) in enumerate(self.points):
                    print("  #%d pos=%s cube=%s" % (i + 1, p, c), flush=True)
            elif k == 'q':
                break
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        with open('/tmp/teach_points.txt', 'w') as f:
            for p, c in self.points:
                f.write("%r %r\n" % (p, c))
        print("已保存 %d 个配对点到 /tmp/teach_points.txt" % len(self.points), flush=True)


if __name__ == '__main__':
    rospy.init_node('teach_pad', anonymous=True)
    try:
        Teach().run()
    except KeyboardInterrupt:
        pass
