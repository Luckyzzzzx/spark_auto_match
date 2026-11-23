#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 目标检测节点（A 组负责）。

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
import numpy as np
import yaml
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

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


def load_model_specs(detect_cfg):
    """把 detect 段解析成模型清单：[{path, backend, aliases}]。

    新写法（推荐，一台配置适配多台机器人）::

        models:
          - path: ~/auto.pt        # 不存在就自动跳过
          - path: ~/yolov8n.pt
            aliases: {"teddy bear": bear}

    旧写法（单模型、全局 backend/class_aliases）仍然兼容。
    """
    specs = []
    models = detect_cfg.get('models')
    if isinstance(models, list) and models:
        for item in models:
            if isinstance(item, dict):
                specs.append({
                    'path': item.get('path'),
                    'backend': str(item.get('backend') or 'auto').lower(),
                    'aliases': {str(k): str(v) for k, v in (item.get('aliases') or {}).items()},
                })
            else:
                specs.append({'path': item, 'backend': 'auto', 'aliases': {}})
        return specs

    backend = str(detect_cfg.get('backend') or 'auto').lower()
    aliases = {str(k): str(v) for k, v in (detect_cfg.get('class_aliases') or {}).items()}
    for key in ('model_auto', 'model_banana', 'model_cake'):
        value = detect_cfg.get(key)
        if value:
            specs.append({'path': value, 'backend': backend, 'aliases': dict(aliases)})
    return specs


class SparkDetect:
    """YOLOv5 / Ultralytics 双后端封装，支持一份配置挂多个模型（按文件存在性自动取舍）。"""

    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.size_x = []
            self.size_y = []
            self.confidence = []
            self.image = None

    def __init__(self, specs, augment=True, iou_thres=0.45, img_size=640):
        self.entries = []   # [(backend, model, aliases)]
        for spec in specs:
            path = _expand(spec.get('path'))
            if not path:
                continue
            if not os.path.isfile(path):
                # 每台机器人权重不同，缺失是常态，不该刷 error
                rospy.loginfo('模型不存在，跳过: %s', path)
                continue
            loaded = self._load(path, spec.get('backend') or 'auto')
            if loaded:
                backend, model = loaded
                self.entries.append((backend, model, spec.get('aliases') or {}))

        if not self.entries:
            # 没有模型还继续跑，只会让下游一直"看不到物体"，不如当场报错
            raise RuntimeError('没有任何可用模型，请检查 items_config.yaml 的 detect.model_*')

        self.augment = bool(augment)
        self.iou_thres = float(iou_thres)
        self.img_size = int(img_size)

        # 所有模型能认出的类别名（用于启动时自检，见 Detector.check_model_classes）
        self.names = set()
        for backend, model, aliases in self.entries:
            for name in self._model_names(backend, model):
                self.names.add(self._alias(name, aliases))

    def summary(self):
        """启动日志用：每个已加载模型一行摘要。"""
        parts = []
        for backend, model, _aliases in self.entries:
            parts.append('%s(%d类)' % (backend, len(self._model_names(backend, model))))
        return ', '.join(parts) or '无'

    def _load(self, path, backend):
        """按 backend 加载；backend=auto 时先试 ultralytics，再回退 yolov5。"""
        order = [backend] if backend in ('ultralytics', 'yolov5') else ['ultralytics', 'yolov5']
        for name in order:
            try:
                if name == 'ultralytics':
                    from ultralytics import YOLO
                    rospy.loginfo('加载模型(ultralytics): %s', path)
                    return 'ultralytics', YOLO(path)
                import yolov5
                rospy.loginfo('加载模型(yolov5): %s', path)
                return 'yolov5', yolov5.load(path)
            except Exception as exc:
                rospy.loginfo('%s 加载失败(%s): %s', name, path, str(exc).splitlines()[0][:110])
        rospy.logerr('模型加载失败，所有后端都不认: %s', path)
        return None

    @staticmethod
    def _alias(name, aliases):
        return aliases.get(str(name), str(name))

    @staticmethod
    def _model_names(backend, model):
        if backend == 'ultralytics':
            names = getattr(model, 'names', {})
        else:
            names = getattr(getattr(model, 'model', None), 'names', {})
        if isinstance(names, dict):
            return list(names.values())
        return list(names or [])

    @staticmethod
    def _name_at(names, index):
        if isinstance(names, dict):
            return names.get(index, names.get(str(index), str(index)))
        return names[index]

    def detect(self, image_bgr, draw=True):
        """检测一帧（输入 BGR），返回结果对象；draw=True 时在原图上画框。

        每个模型用自己的后端和别名：ultralytics 吃 BGR，yolov5 封装吃 RGB，
        这里按模型分别转换，画框统一回到 BGR 画布上。
        """
        result = self.__results__()

        for backend, model, aliases in self.entries:
            source = image_bgr if backend == 'ultralytics' else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            if backend == 'ultralytics':
                try:
                    output = model.predict(
                        source=source,
                        imgsz=self.img_size,
                        augment=self.augment,
                        iou=self.iou_thres,
                        verbose=False,
                    )[0]
                    names = output.names
                    boxes = [] if output.boxes is None else output.boxes
                except Exception as exc:
                    rospy.logerr_throttle(2.0, 'Ultralytics 推理失败: %s', exc)
                    continue
            else:
                try:
                    try:
                        outputs = model(source, size=self.img_size,
                                        augment=self.augment, iou=self.iou_thres)
                    except TypeError:
                        # 部分 yolov5 版本 forward() 不接受 iou 参数，回退到默认
                        outputs = model(source, size=self.img_size, augment=self.augment)
                    names = model.model.names
                    boxes = outputs.xyxy[0]
                except Exception as exc:
                    rospy.logerr_throttle(2.0, 'YOLOv5 推理失败: %s', exc)
                    continue

            for box in boxes:
                if backend == 'ultralytics':
                    xyxy = box.xyxy[0].tolist()
                    conf = float(box.conf[0].item())
                    cls = int(box.cls[0].item())
                else:
                    values = list(box)
                    xyxy, conf, cls = values[:4], float(values[4]), int(values[5])
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                size_x = int(x2 - x1)
                size_y = int(y2 - y1)
                name = self._alias(self._name_at(names, int(cls)), aliases)

                if draw:
                    label = '%s %.2f' % (name, float(conf))
                    cv2.rectangle(image_bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
                    cv2.putText(image_bgr, label, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.circle(image_bgr, (center_x, center_y), 5, (255, 0, 0), -1)

                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(name)
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))

        result.image = image_bgr
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
        self._fps_frames = 0
        self._fps_t0 = time.time()

        self.image_pub = rospy.Publisher("/debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()

        specs = load_model_specs(detect_cfg)
        self.detector = SparkDetect(
            specs,
            augment=detect_cfg.get('augment', True),
            iou_thres=detect_cfg.get('iou_thres', 0.45),
            img_size=detect_cfg.get('img_size', 640),
        )

        image_topic = rospy.get_param('~image_topic', 'image')
        self.image_sub = rospy.Subscriber(
            image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24
        )
        self.check_model_classes(detect_cfg.get('check_classes') or [])
        rospy.loginfo('detect_ros 就绪: models=[%s] image=%s conf=%.2f class_conf=%s filter_ids=%s max_rate=%sHz',
                      self.detector.summary(), image_topic, self.conf_thres,
                      self.class_conf, sorted(self.filter_ids),
                      '不限' if not self.min_period else '%.1f' % (1.0 / self.min_period))

    def check_model_classes(self, configured):
        """启动自检：配置里要用到的类别，模型到底认不认。

        典型坑：规则里第四类物品是「蛋糕(cake)」，但权重是上一版训的（只有 wine），
        缺了不会报错，只会在比赛里表现为「二层那块永远抓不到」，所以这里提前喊出来。
        """
        wanted = list(configured) or [
            name for name in self.items
            if not name.endswith('_site') and name != 'base'
        ]
        missing = [name for name in wanted if name not in self.detector.names]
        if missing:
            rospy.logwarn('模型缺少这些类别，对应物品永远无法识别: %s（模型实际有: %s）',
                          missing, sorted(self.detector.names))
            for name in missing:
                if str(name) in self.class_conf:
                    rospy.logwarn('  class_conf 里为 %r 设了阈值，但模型没有这个类，设置无效', name)
        else:
            rospy.loginfo('模型类别自检通过: %s', wanted)

    def _threshold_for(self, name):
        return self.class_conf.get(name, self.conf_thres)

    @staticmethod
    def _iou(a, b):
        ax1 = a['cx'] - a['w'] / 2.0
        ay1 = a['cy'] - a['h'] / 2.0
        ax2 = a['cx'] + a['w'] / 2.0
        ay2 = a['cy'] + a['h'] / 2.0
        bx1 = b['cx'] - b['w'] / 2.0
        by1 = b['cy'] - b['h'] / 2.0
        bx2 = b['cx'] + b['w'] / 2.0
        by2 = b['cy'] + b['h'] / 2.0
        ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
        iy = max(0.0, min(ay2, by2) - max(ay1, by1))
        inter = ix * iy
        if inter <= 0:
            return 0.0
        union = (a['w'] * a['h']) + (b['w'] * b['h']) - inter
        return inter / union if union > 0 else 0.0

    def _merge_duplicates(self, candidates, iou_thres=0.5):
        """两个模型都检出同一块金砖时，只保留分数高的那个。

        金砖一层是 3x3、间隔 4.5cm，重复框会让下游按 y 排序时反复挑同一个位置。
        类别不同但框高度重叠（例如 bear/wine 混淆）也会合并，并打一条 warn 方便回头查。
        """
        if len(candidates) < 2:
            return candidates
        ordered = sorted(candidates, key=lambda c: c['score'], reverse=True)
        kept = []
        for cand in ordered:
            clash = None
            for other in kept:
                if self._iou(cand, other) >= iou_thres:
                    clash = other
                    break
            if clash is not None:
                if cand['id'] != clash['id']:
                    rospy.logwarn_throttle(2.0, '同一位置被识别成两类: %s(%.2f) vs %s(%.2f)，保留高分',
                                           cand['name'], cand['score'], clash['name'], clash['score'])
                continue
            kept.append(cand)
        return kept

    def _log_fps(self, n_detections):
        """每 5 秒打一次帧率，方便现场判断要不要关 augment / 降 img_size。"""
        self._fps_frames += 1
        elapsed = time.time() - self._fps_t0
        if elapsed >= 5.0:
            rospy.loginfo('检测帧率 %.1f fps（上次统计后 %d 帧），本帧 %d 个目标',
                          self._fps_frames / elapsed, self._fps_frames, n_detections)
            self._fps_frames = 0
            self._fps_t0 = time.time()

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

        # 统一按 BGR 交给 SparkDetect，内部再按每个模型的后端决定是否转 RGB
        image = cv_image.copy()

        obj_array = Detection2DArray()
        obj_array.header = data.header

        try:
            results = self.detector.detect(image, draw=self.publish_debug)
        except Exception as exc:
            rospy.logerr_throttle(2.0, '检测异常，本帧发布空结果: %s', exc)
            self.object_pub.publish(obj_array)
            return

        dropped = 0
        candidates = []
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
            candidates.append({
                'id': item_id,
                'name': name,
                'score': float(results.confidence[i]),
                'cx': int(results.x[i]),
                'cy': int(results.y[i]),
                'w': int(results.size_x[i]),
                'h': int(results.size_y[i]),
            })

        # 两个模型会各检出同一块金砖（一层 3x3，间隔才 4.5cm，重复框会害抓取选错目标）
        candidates = self._merge_duplicates(candidates)

        for cand in candidates:
            obj = Detection2D()
            obj.header = data.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = cand['id']
            hypothesis.score = cand['score']
            obj.results.append(hypothesis)
            obj.bbox.size_y = cand['h']
            obj.bbox.size_x = cand['w']
            obj.bbox.center.x = cand['cx']
            obj.bbox.center.y = cand['cy']
            obj_array.detections.append(obj)

        if dropped:
            rospy.logdebug_throttle(2.0, '本帧丢弃 %d 个框（非目标/低置信度/filter_ids）', dropped)

        self._log_fps(len(obj_array.detections))

        self.object_pub.publish(obj_array)

        if not self.publish_debug:
            return
        try:
            img = results.image          # SparkDetect 已经统一返回 BGR
            # 机器人视觉 venv 的 OpenCV 5 与 Noetic cv_bridge 的类型表不兼容，
            # cv2_to_imgmsg 会在 CV_8UC3(16) 上抛 KeyError。这里按 ROS Image
            # 协议直接构造消息，避免调试图拖垮整个检测回调。
            img = np.ascontiguousarray(img, dtype=np.uint8)
            if img.ndim != 3 or img.shape[2] != 3:
                raise ValueError("调试图必须是 HxWx3，实际 %r" % (img.shape,))
            image_out = Image()
            image_out.height = int(img.shape[0])
            image_out.width = int(img.shape[1])
            image_out.encoding = "bgr8"
            image_out.is_bigendian = 0
            image_out.step = int(img.shape[1] * 3)
            image_out.data = img.tobytes()
            image_out.header = data.header
            self.image_pub.publish(image_out)
        except Exception as exc:
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
