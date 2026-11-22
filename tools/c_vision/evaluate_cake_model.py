"""Read-only evaluation helper for the robot's YOLOv5 cake trial model."""

import argparse
from pathlib import Path

import cv2
import yolov5


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("weights")
    parser.add_argument("images", nargs="+")
    parser.add_argument("--size", type=int, default=640)
    args = parser.parse_args()
    model = yolov5.load(args.weights)
    model.conf = 0.05  # Report weak candidates too; deployment still uses 0.5.
    print("classes:", model.model.names, flush=True)
    for path in map(Path, args.images):
        image = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        results = model(image, size=args.size)
        detections = [
            [round(float(v), 3) for v in row]
            for row in results.xyxy[0].cpu().numpy()
        ]
        print(path.name, detections, flush=True)


if __name__ == "__main__":
    main()
