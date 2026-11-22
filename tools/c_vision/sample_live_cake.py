#!/usr/bin/env python3
"""Sample consecutive ROS frames to audit cake detections without arm motion."""

import pathlib
import time

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import torch
import yolov5


pathlib.WindowsPath = pathlib.PosixPath
torch.set_num_threads(2)
model = yolov5.load("/home/spark/cake_best.pt")
model.conf = 0.1
rospy.init_node("cake_live_audit", anonymous=True)
bridge = CvBridge()
seen = set()
for i in range(10):
    while True:
        msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
        stamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        if stamp not in seen:
            break
    seen.add(stamp)
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    output = model(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), size=416).xyxy[0].cpu().numpy()
    cakes = [[round(float(v), 2) for v in row[:5]] for row in output
             if model.model.names[int(row[5])] == "cake"]
    print(i, cakes, flush=True)
    time.sleep(.3)
