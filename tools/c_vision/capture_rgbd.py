#!/usr/bin/env python3
"""Capture one synchronized color/aligned-depth frame and color intrinsics."""

import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


def main():
    rospy.init_node("cube_rgbd_capture", anonymous=True)
    folder = Path("/tmp/cube_rgbd")
    folder.mkdir(exist_ok=True)
    camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=10)
    bridge = CvBridge()
    color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.08)
    captured = []

    def on_frame(color_msg, depth_msg):
        if captured:
            return
        color = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        cv2.imwrite(str(folder / "color.png"), color)
        np.save(str(folder / "depth.npy"), depth)
        valid = depth[np.isfinite(depth) & (depth > 0)]
        info = {"color_shape": list(color.shape), "depth_shape": list(depth.shape),
                "depth_encoding": depth_msg.encoding, "depth_dtype": str(depth.dtype),
                "depth_min": float(valid.min()) if valid.size else None,
                "depth_max": float(valid.max()) if valid.size else None,
                "K": list(camera_info.K),
                "color_stamp": color_msg.header.stamp.to_sec(),
                "depth_stamp": depth_msg.header.stamp.to_sec()}
        (folder / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
        print(json.dumps(info), flush=True)
        captured.append(True)

    sync.registerCallback(on_frame)
    deadline = rospy.Time.now() + rospy.Duration(10)
    while not rospy.is_shutdown() and not captured and rospy.Time.now() < deadline:
        rospy.sleep(0.05)
    if not captured:
        raise SystemExit("No synchronized RGB-D frame")


if __name__ == "__main__":
    main()
