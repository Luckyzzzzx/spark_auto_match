#!/usr/bin/env python3
"""Audit cake detection gated by a complete upper-layer depth top face."""

import json
import pathlib
import time

import cv2
from cv_bridge import CvBridge
import message_filters
import rospy
from sensor_msgs.msg import CameraInfo, Image
import torch
import yolov5

from cube_top_geometry import find_top_faces


pathlib.WindowsPath = pathlib.PosixPath
torch.set_num_threads(2)
model = yolov5.load("/home/spark/cake_best.pt")
model.conf = .1
rospy.init_node("cake_rgbd_audit", anonymous=True)
info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=10)
bridge = CvBridge()
color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, .08)
rows = []


def on_frame(color_msg, depth_msg):
    if len(rows) >= 10:
        return
    color = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
    boxes = model(cv2.cvtColor(color, cv2.COLOR_BGR2RGB), size=416).xyxy[0].cpu().numpy()
    cakes = [box for box in boxes if model.model.names[int(box[5])] == "cake"]
    try:
        faces, floor_error = find_top_faces(depth, info.K)
    except ValueError as exc:
        faces, floor_error = [], None
        print("geometry error:", exc, flush=True)
    upper = [face for face in faces if face["level"] == "upper"]
    matches = []
    for box in cakes:
        x1, y1, x2, y2, score, _ = box
        for face in upper:
            u, v = face["center_uv"]
            if score >= .45 and x1 <= u <= x2 and y1 <= v <= y2:
                matches.append({"score": round(float(score), 3),
                                "center_uv": face["center_uv"],
                                "corners_uv": face["corners_uv"]})
    result = {"frame": len(rows), "cake_scores": [round(float(b[4]), 3) for b in cakes],
              "upper_faces": len(upper), "matches": matches,
              "floor_error_mm_95": None if floor_error is None else round(floor_error * 1000, 2)}
    rows.append(result)
    print(json.dumps(result, ensure_ascii=False), flush=True)


sync.registerCallback(on_frame)
deadline = time.monotonic() + 60
while not rospy.is_shutdown() and len(rows) < 10 and time.monotonic() < deadline:
    rospy.sleep(.1)
if len(rows) < 10:
    raise SystemExit("Only received %d synchronized frames" % len(rows))
