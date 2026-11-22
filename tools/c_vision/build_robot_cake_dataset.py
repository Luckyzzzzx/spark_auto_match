"""Auto-label the robot's cake-prop capture sessions for YOLOv5.

The distinctive red print is only used to propose boxes. Review the generated
previews before training; this is a scene-specific labelling aid.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def locate_cake(image):
    # Red cake decoration, excluding the orange robot marker (G-B is larger).
    blue, green, red = cv2.split(image.astype(np.int16))
    mask = ((red - green > 23) & (red - blue > 23)
            & (green - blue < 23) & (red > 80)).astype(np.uint8)
    spread = cv2.dilate(mask, np.ones((33, 33), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(spread)
    candidates = []
    for component in range(1, count):
        x, y, w, h, _ = stats[component]
        red_pixels = int(np.count_nonzero(mask[labels == component]))
        if red_pixels >= 300 and w >= 40 and h >= 40:
            candidates.append((red_pixels, (x, y, x + w, y + h)))
    if not candidates:
        return None, 0
    score, (x1, y1, x2, y2) = max(candidates)
    height, width = image.shape[:2]
    return (max(0, x1 - 5), max(0, y1 - 5),
            min(width, x2 + 5), min(height, y2 + 5)), score


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Directory containing gold_photos_cake_* sessions")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sessions = sorted(args.source.glob("gold_photos_cake_*"))
    if len(sessions) < 2:
        raise SystemExit("Need at least two capture sessions for a session-level split")
    for split in ("train", "val"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)
    (args.output / "preview").mkdir(parents=True, exist_ok=True)
    counts = {"train": 0, "val": 0, "skipped": 0}
    for session_index, session in enumerate(sessions):
        split = "val" if session_index == len(sessions) - 1 else "train"
        for source in sorted(session.glob("cake_group_*_photo_*.png")):
            image = cv2.imread(str(source))
            if image is None:
                raise ValueError(f"Cannot read {source}")
            box, red_pixels = locate_cake(image)
            if box is None:
                counts["skipped"] += 1
                print("SKIP", source, flush=True)
                continue
            height, width = image.shape[:2]
            x1, y1, x2, y2 = box
            name = f"session{session_index + 1}_{source.stem}"
            dest = args.output / "images" / split / f"{name}.png"
            if not dest.exists():
                dest.symlink_to(source)
            values = ((x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
                      (x2 - x1) / width, (y2 - y1) / height)
            (args.output / "labels" / split / f"{name}.txt").write_text(
                "0 " + " ".join(f"{v:.6f}" for v in values) + "\n"
            )
            counts[split] += 1
            if source.stem.endswith("photo_01"):
                preview = image.copy()
                cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(preview, str(red_pixels), (x1, max(20, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.imwrite(str(args.output / "preview" / f"{name}.jpg"), preview)
    (args.output / "data.yaml").write_text(
        f"path: {args.output.resolve()}\ntrain: images/train\nval: images/val\nnc: 1\nnames: [cake]\n"
    )
    print(counts, flush=True)


if __name__ == "__main__":
    main()
