#!/usr/bin/env python3
"""ROS Noetic node: detect the printed cake cube from the D435 color stream.

Publishes /cake_detections (vision_msgs/Detection2DArray) and /cake_debug_image.
Run it alongside the existing detector without taking over /objects.
"""

import os
import time

import cv2
import rospy
import torch
import yolov5
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class CakeDetector:
    def __init__(self):
        model_path = rospy.get_param(
            "~model_path",
            "/home/spark/cake_dataset_v4_20261122/runs/cake_v4/weights/best.pt",
        )
        if not os.path.isfile(model_path):
            raise FileNotFoundError(model_path)
        torch.set_num_threads(int(rospy.get_param("~torch_threads", 2)))
        self.model = yolov5.load(model_path)
        names = self.model.model.names
        if set(names.values()) != {"cake"}:
            raise ValueError("Expected a one-class cake model; got %r" % names)
        self.model.conf = float(rospy.get_param("~confidence", 0.5))
        self.image_size = int(rospy.get_param("~image_size", 416))
        self.object_id = int(rospy.get_param("~object_id", 5))
        self.min_period = 1.0 / max(0.1, float(rospy.get_param("~max_rate_hz", 5.0)))
        self.last_inference_time = 0.0
        self.bridge = CvBridge()
        self.detections_pub = rospy.Publisher(
            "/cake_detections", Detection2DArray, queue_size=1
        )
        self.debug_pub = rospy.Publisher("/cake_debug_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.on_image,
            queue_size=1, buff_size=2**24,
        )
        rospy.loginfo("Cake detector ready: %s", model_path)

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_inference_time < self.min_period:
            return
        self.last_inference_time = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, "bgr8")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            predictions = self.model(rgb, size=self.image_size).xyxy[0]
            output = Detection2DArray()
            output.header = message.header
            debug = bgr.copy() if self.debug_pub.get_num_connections() else None
            for x1, y1, x2, y2, confidence, class_id in predictions:
                if self.model.model.names[int(class_id)] != "cake":
                    continue
                detection = Detection2D()
                detection.header = message.header
                detection.bbox.center.x = float((x1 + x2) / 2)
                detection.bbox.center.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = self.object_id
                hypothesis.score = float(confidence)
                detection.results.append(hypothesis)
                output.detections.append(detection)
                if debug is not None:
                    cv2.rectangle(debug, (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 255, 0), 2)
                    cv2.putText(debug, "cake %.2f" % confidence,
                                (int(x1), max(15, int(y1) - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.detections_pub.publish(output)
            if debug is not None:
                debug_message = self.bridge.cv2_to_imgmsg(debug, "bgr8")
                debug_message.header = message.header
                self.debug_pub.publish(debug_message)
        except Exception as error:
            rospy.logerr_throttle(5, "Cake detection failed: %s" % error)


if __name__ == "__main__":
    rospy.init_node("cake_detector")
    CakeDetector()
    rospy.spin()
