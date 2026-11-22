"""Evaluate cake recall and non-cake false positives at the deploy threshold."""

import argparse
from pathlib import Path

import cv2
import yolov5


def iou(a, b):
    left = max(a[0], b[0])
    top = max(a[1], b[1])
    right = min(a[2], b[2])
    bottom = min(a[3], b[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return intersection / max(1, area_a + area_b - intersection)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("weights")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()
    model = yolov5.load(args.weights)
    model.conf = args.confidence
    positives = hits = negatives = false_positive_images = 0
    misses = []
    false_positives = []
    for image_path in sorted((args.dataset / "images" / "val").glob("*.png")):
        image = cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)
        height, width = image.shape[:2]
        label_path = args.dataset / "labels" / "val" / f"{image_path.stem}.txt"
        label = label_path.read_text().strip()
        boxes = model(image, size=416).xyxy[0].cpu().numpy()
        if not label:
            negatives += 1
            if len(boxes):
                false_positive_images += 1
                false_positives.append((image_path.name, round(float(boxes[:, 4].max()), 3)))
            continue
        positives += 1
        _, cx, cy, bw, bh = map(float, label.split())
        true_box = ((cx - bw / 2) * width, (cy - bh / 2) * height,
                    (cx + bw / 2) * width, (cy + bh / 2) * height)
        best = max((iou(row[:4], true_box) for row in boxes), default=0)
        if best >= 0.5:
            hits += 1
        else:
            misses.append((image_path.name, round(best, 3)))
    print(f"Positive images: {positives}; IoU>=0.5 hits: {hits}; misses: {misses}")
    print(f"Negative images: {negatives}; false positive images: {false_positive_images}")
    print("False positives:", false_positives)


if __name__ == "__main__":
    main()
