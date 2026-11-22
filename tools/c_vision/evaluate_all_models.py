"""Inspect the existing separate YOLOv5 weights on a captured robot frame."""

import argparse
import pathlib

import cv2
import yolov5


pathlib.WindowsPath = pathlib.PosixPath


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("weights", nargs="+")
    parser.add_argument("--size", type=int, default=416)
    parser.add_argument("--confidence", type=float, default=0.1)
    args = parser.parse_args()
    image = cv2.cvtColor(cv2.imread(args.image), cv2.COLOR_BGR2RGB)
    for weight in args.weights:
        model = yolov5.load(weight)
        model.conf = args.confidence
        rows = model(image, size=args.size).xyxy[0].cpu().numpy()
        print(weight)
        for row in rows:
            x1, y1, x2, y2, conf, index = row
            print(
                model.model.names[int(index)], round(float(conf), 3),
                tuple(round(float(x), 1) for x in (x1, y1, x2, y2)),
            )


if __name__ == "__main__":
    main()
