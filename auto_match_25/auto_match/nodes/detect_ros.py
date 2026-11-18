#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOv5 目标检测节点（A 组负责）。

对外契约 **没有变化**（下游 runtask.py / board_classifier.py 不用改）：

    订阅  /camera/color/image_raw  (sensor_msgs/Image)
    发布  /objects                 (vision_msgs/Detection2DArray)
          /debug_image             (sensor_msgs/Image，可选)

相比旧版的改动（都属于 A 组自己的文件）：
  1. 阈值/模型路径/推理参数全部改到 config/items_config.yaml，调参不用改代码；
  2. 支持按类别单独设阈值（class_conf），分类板小图案能单独放宽；
  3. `/objects` 多了按 `filter_ids` 过滤的开关（默认不过滤，保持旧行为）；
  4. 双模型推理加帧率上限（max_rate_hz），避免消息积压导致下游拿到过期帧；
  5. 模型缺失 / 图像转换失败时不再静默 `except: pass`，而是明确报错；
  6. 调试图可关（publish_debug_image），省带宽。
"""

import os
import sys
import time

import cv2
import yaml
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import yolov5

import platform
import pathlib

plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

CONFIG_REL_PATH = os.path.join('config', 'items_config.yaml')


def _expand(path):
    """展开 ~ 和 $HOME，空值返回 None。"""
    if not path:
        return None
    return os.path.expanduser(os.path.expandvars(str(path)))


def load_config():
    """读取 items_config.yaml，失败直接抛异常（宁可启动失败，也不要带病上场）。"""
    try:
        package_path = rospkg.RosPack().get_path('auto_match')
        cfg_path = os.path.join(package_path, CONFIG_REL_PATH)
    except Exception as exc:
        raise RuntimeError('找不到 auto_match 功能包: %s' % exc)

    if not os.path.isfile(cfg_path):
        raise RuntimeError('缺少配置文件: %s' % cfg_path)

    with open(cfg_path, 'r', encoding='utf8') as fp:
        content = yaml.safe_load(fp.read()) or {}

    if not content.get('items'):
        raise RuntimeError('items_config.yaml 里 items 段为空: %s' % cfg_path)
    return content


class SparkDetect:
    """YOLOv5 双模型封装（不改动模型本身的推理方式）。"""

    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.size_x = []
            self.size_y = []
            self.confidence = []
            self.image = None

    def __init__(self, model_paths, augment=True, iou_thres=0.45, img_size=640):
        self.models = []
        for path in model_paths:
            path = _expand(path)
            if not path:
                continue
            if not os.path.isfile(path):
                rospy.logerr('模型文件不存在，跳过: %s', path)
                continue
            try:
                rospy.loginfo('加载模型: %s', path)
                self.models.append(yolov5.load(path))
            except Exception as exc:
                rospy.logerr('模型加载失败 %s: %s', path, exc)

        if not self.models:
            # 没有模型还继续跑，只会让下游一直"看不到物体"，不如当场报错
            raise RuntimeError('没有任何可用模型，请检查 items_config.yaml 的 detect.model_*')

        self.augment = bool(augment)
        self.iou_thres = float(iou_thres)
        self.img_size = int(img_size)

    def detect(self, image, draw=True):
        """检测一帧，返回结果对象；image 会被就地画框（draw=True 时）。"""
        result = self.__results__()

        for model in self.models:
            try:
                outputs = model(image, size=self.img_size,
                                augment=self.augment, iou=self.iou_thres)
            except Exception as exc:
                rospy.logerr_throttle(2.0, '推理失败: %s', exc)
                continue

            names = model.model.names
            try:
                boxes = outputs.xyxy[0]
            except Exception:
                continue

            for *xyxy, conf, cls in boxes:
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                size_x = int(x2 - x1)
                size_y = int(y2 - y1)

                if draw:
                    label = '%s %.2f' % (names[int(cls)], float(conf))
                    cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
                    cv2.putText(image, label, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))

        result.image = image
        return result


class Detector:
    def __init__(self):
        content = load_config()
        self.obj_id = content['items']
        self.items = set(self.obj_id.keys())

        detect_cfg = content.get('detect') or {}
        self.conf_thres = float(detect_cfg.get('conf_thres', 0.5))
        self.class_conf = {str(k): float(v) for k, v in (detect_cfg.get('class_conf') or {}).items()}
        self.filter_ids = set(int(i) for i in (detect_cfg.get('filter_ids') or []))
        self.publish_debug = bool(detect_cfg.get('publish_debug_image', True))
        max_rate = float(detect_cfg.get('max_rate_hz', 0.0) or 0.0)
        self.min_period = (1.0 / max_rate) if max_rate > 0 else 0.0
        self._last_stamp = 0.0

        self.image_pub = rospy.Publisher("/debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()

        self.detector = SparkDetect(
            [detect_cfg.get('model_auto', '~/auto.pt'),
             detect_cfg.get('model_banana', '~/banana.pt')],
            augment=detect_cfg.get('augment', True),
            iou_thres=detect_cfg.get('iou_thres', 0.45),
            img_size=detect_cfg.get('img_size', 640),
        )

        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2 ** 24
        )
        rospy.loginfo('detect_ros 就绪: conf=%.2f class_conf=%s filter_ids=%s max_rate=%sHz',
                      self.conf_thres, self.class_conf, sorted(self.filter_ids),
                      '不限' if not self.min_period else '%.1f' % (1.0 / self.min_period))

    def _threshold_for(self, name):
        return self.class_conf.get(name, self.conf_thres)

    def image_cb(self, data):
        # ---- 主动限帧：双模型推理慢，丢帧比排队更好（排队会让下游拿到旧图） ----
        now = time.time()
        if self.min_period and (now - self._last_stamp) < self.min_period:
            return
        self._last_stamp = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, '图像转换失败: %s', exc)
            return
        except Exception as exc:
            rospy.logerr_throttle(2.0, '图像消息异常: %s', exc)
            return

        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        obj_array = Detection2DArray()
        obj_array.header = data.header

        try:
            results = self.detector.detect(image, draw=self.publish_debug)
        except Exception as exc:
            rospy.logerr_throttle(2.0, '检测异常，本帧发布空结果: %s', exc)
            self.object_pub.publish(obj_array)
            return

        dropped = 0
        for i in range(len(results.name)):
            name = results.name[i]
            if name not in self.items:
                dropped += 1
                continue
            item_id = int(self.obj_id[name])
            if item_id in self.filter_ids:
                dropped += 1
                continue
            if results.confidence[i] < self._threshold_for(name):
                dropped += 1
                continue

            obj = Detection2D()
            obj.header = data.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = item_id
            hypothesis.score = results.confidence[i]
            obj.results.append(hypothesis)
            obj.bbox.size_y = int(results.size_y[i])
            obj.bbox.size_x = int(results.size_x[i])
            obj.bbox.center.x = int(results.x[i])
            obj.bbox.center.y = int(results.y[i])
            obj_array.detections.append(obj)

        if dropped:
            rospy.logdebug_throttle(2.0, '本帧丢弃 %d 个框（非目标/低置信度/filter_ids）', dropped)

        self.object_pub.publish(obj_array)

        if not self.publish_debug:
            return
        try:
            img = cv2.cvtColor(results.image, cv2.COLOR_RGB2BGR)
            image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
            image_out.header = data.header
            self.image_pub.publish(image_out)
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, '调试图发布失败: %s', exc)


if __name__ == '__main__':
    rospy.init_node('detector_node')
    try:
        obj = Detector()
    except Exception as exc:
        rospy.logfatal('detect_ros 启动失败: %s', exc)
        sys.exit(2)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()
