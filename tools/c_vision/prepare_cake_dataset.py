"""Build the small, manually annotated cake-prop example dataset.

These photos depict a printed cake on a cube. The label describes the whole
cube, not edible cake. Add varied, independently captured images before
training a production detector.
"""

from pathlib import Path
from shutil import copyfile

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
SOURCE = Path(r"C:\Users\华硕\AppData\Local\Temp")
DATA = ROOT / "cake_dataset"

# filename, source image, pixel bounding box of the cake cube
SAMPLES = [
    ("cake_01", "codex-clipboard-b02e9028-aa60-4bb8-a942-ab3f5185118b.png", (749, 379, 954, 584)),
    ("cake_02", "codex-clipboard-ec6a442f-c48a-47db-a4da-f60eba137e96.png", (728, 367, 930, 576)),
    ("cake_03", "codex-clipboard-5f61b5e5-5864-49de-9428-68203a5bdd68.png", (615, 294, 799, 512)),
    ("cake_04", "codex-clipboard-c4e142b0-e692-48e9-99b1-de47a923b68b.png", (410, 136, 824, 564)),
]


def main() -> None:
    for split in ("train", "val"):
        (DATA / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATA / "labels" / split).mkdir(parents=True, exist_ok=True)
    (DATA / "preview").mkdir(parents=True, exist_ok=True)

    for index, (name, source_name, box) in enumerate(SAMPLES):
        source = SOURCE / source_name
        if not source.is_file():
            raise FileNotFoundError(f"Missing supplied photo: {source}")
        # A held-out frame is only a format check: four same-scene images
        # cannot provide a meaningful generalization estimate.
        split = "val" if index == 2 else "train"
        dest = DATA / "images" / split / f"{name}.png"
        copyfile(source, dest)
        with Image.open(dest) as im:
            width, height = im.size
            x1, y1, x2, y2 = box
            assert 0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height
            center_x = (x1 + x2) / (2 * width)
            center_y = (y1 + y2) / (2 * height)
            box_w = (x2 - x1) / width
            box_h = (y2 - y1) / height
            label = f"0 {center_x:.6f} {center_y:.6f} {box_w:.6f} {box_h:.6f}\n"
            (DATA / "labels" / split / f"{name}.txt").write_text(label, encoding="utf-8")
            preview = im.convert("RGB")
            draw = ImageDraw.Draw(preview)
            draw.rectangle(box, outline="red", width=4)
            preview.save(DATA / "preview" / f"{name}.jpg", quality=90)

    (DATA / "dataset.yaml").write_text(
        f"path: {DATA.as_posix()}\ntrain: images/train\nval: images/val\n"
        "names:\n  0: cake\n",
        encoding="utf-8",
    )
    print(f"Wrote {len(SAMPLES)} distinct annotated photos to {DATA}")


if __name__ == "__main__":
    main()
