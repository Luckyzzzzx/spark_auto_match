"""Add other competition props as explicit no-cake images for YOLOv5."""

import argparse
from pathlib import Path
from shutil import copyfile


def link_split(base, output, split):
    for kind in ("images", "labels"):
        dest_dir = output / kind / split
        dest_dir.mkdir(parents=True, exist_ok=True)
        for source in (base / kind / split).glob("*"):
            if source.suffix not in (".png", ".txt"):
                continue
            dest = dest_dir / source.name
            if kind == "images":
                if not dest.exists():
                    dest.symlink_to(source.resolve())
            else:
                copyfile(source, dest)


def add_session(session, kind, output, split, first_photo_only):
    photos = sorted(session.glob(f"{kind}_group_*_photo_*.png"))
    if first_photo_only:
        photos = [p for p in photos if p.stem.endswith("photo_01")]
    if not photos:
        raise ValueError(f"No photos in {session}")
    for photo in photos:
        name = f"negative_{kind}_{session.name}_{photo.stem}"
        dest = output / "images" / split / f"{name}.png"
        if not dest.exists():
            dest.symlink_to(photo.resolve())
        (output / "labels" / split / f"{name}.txt").write_text("")
    return len(photos)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("captures", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    for split in ("train", "val"):
        link_split(args.base, args.output, split)
    for kind in ("banana", "clock", "doll"):
        sessions = sorted(args.captures.glob(f"gold_photos_{kind}_*"))
        if len(sessions) < 2:
            raise ValueError(f"Need two independent {kind} sessions")
        train_count = add_session(sessions[0], kind, args.output, "train", False)
        val_count = add_session(sessions[-1], kind, args.output, "val", True)
        print(kind, "negative train:", train_count, "val:", val_count)
    (args.output / "data.yaml").write_text(
        f"path: {args.output.resolve()}\ntrain: images/train\nval: images/val\n"
        "nc: 1\nnames: [cake]\n"
    )
    print("Dataset ready:", args.output)


if __name__ == "__main__":
    main()
