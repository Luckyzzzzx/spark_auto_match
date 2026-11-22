#!/usr/bin/env python3
"""Inspect top-face quadrilateral candidates on the captured mixed-cube frame."""

import json
import sys

import cv2
import numpy as np

sys.path.insert(0, "/tmp")
from cake_corners_eval import auto_candidates, diagonal_center


image = cv2.imread("/tmp/cake_pose2_20261122.png")
if image is None:
    raise SystemExit("Missing captured frame")

boxes = [
    ("cake", (250, 19, 407, 239)),
    ("bear_top_left", (118, 32, 221, 106)),
    ("clock_left", (93, 138, 198, 227)),
    ("bear_right", (423, 119, 530, 214)),
    ("bear_lower_left", (122, 267, 230, 380)),
    ("clock_lower_middle", (295, 268, 406, 380)),
    ("banana_lower_right", (448, 245, 570, 360)),
]

report = []
overlay = image.copy()
for name, (x1, y1, x2, y2) in boxes:
    hint = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
    roi = (max(0, x1 - 15), max(0, y1 - 15),
           x2 - x1 + 30, y2 - y1 + 30)
    candidates = auto_candidates(image, hint, roi)
    entry = {"name": name, "bbox": [x1, y1, x2, y2],
             "num_candidates": len(candidates)}
    if candidates:
        score, points = candidates[0]
        center = diagonal_center(points)
        entry.update(score=round(float(score), 3),
                     corners=points.round(1).tolist(),
                     center=center.round(1).tolist())
        cv2.polylines(overlay, [np.rint(points).astype(np.int32)], True,
                      (0, 255, 0), 2)
        cv2.drawMarker(overlay, tuple(np.rint(center).astype(int)),
                       (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
    cv2.putText(overlay, name, (x1, max(y1 - 4, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    report.append(entry)

cv2.imwrite("/tmp/pose2_corner_candidates.png", overlay)
print(json.dumps(report, ensure_ascii=False, indent=2))
