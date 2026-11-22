"""Make a fine-tuning dataset with extra copies of the supplied viewpoints.

Repeated files receive independent YOLO augmentations each epoch. The held-out
robot capture session from v2 stays in validation.
"""

import argparse
from pathlib import Path
from shutil import copyfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("user_images", type=Path)
    parser.add_argument("user_labels", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--repeats", type=int, default=10)
    args = parser.parse_args()

    for split in ("train", "val"):
        for kind in ("images", "labels"):
            dest_dir = args.output / kind / split
            dest_dir.mkdir(parents=True, exist_ok=True)
            for source in (args.base / kind / split).glob("*"):
                if source.suffix not in (".png", ".txt"):
                    continue
                dest = dest_dir / source.name
                if kind == "images":
                    if not dest.exists():
                        dest.symlink_to(source.resolve())
                else:
                    copyfile(source, dest)

    for image in sorted(args.user_images.glob("cake_*.png")):
        label = args.user_labels / f"{image.stem}.txt"
        if not label.is_file():
            raise FileNotFoundError(label)
        for repeat in range(args.repeats):
            name = f"user_{image.stem}_r{repeat:02d}"
            dest = args.output / "images" / "train" / f"{name}.png"
            if not dest.exists():
                dest.symlink_to(image.resolve())
            copyfile(label, args.output / "labels" / "train" / f"{name}.txt")

    (args.output / "data.yaml").write_text(
        f"path: {args.output.resolve()}\ntrain: images/train\nval: images/val\n"
        "nc: 1\nnames: [cake]\n"
    )
    print("Fine-tuning dataset ready:", args.output)


if __name__ == "__main__":
    main()
