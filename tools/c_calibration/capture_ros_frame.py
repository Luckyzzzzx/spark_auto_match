#!/usr/bin/env python3
"""Save one ROS color frame for visual calibration."""

import argparse

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--output", default="/tmp/c_camera_frame.png")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    rospy.init_node("capture_calibration_frame", anonymous=True)
    message = rospy.wait_for_message(args.topic, Image, timeout=args.timeout)
    frame = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
    if not cv2.imwrite(args.output, frame):
        raise RuntimeError("Could not save image: " + args.output)
    print("Saved", args.output, "size", frame.shape[1], "x", frame.shape[0])


if __name__ == "__main__":
    main()
