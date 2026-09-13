"""
generate_placeholder_images.py

Creates one labeled placeholder PNG per unique (crop, growth_stage,
diseased) combination referenced by pod_registry.PODS.

THESE ARE NOT REAL DATA. They're flat-colored tiles with the pod's
ground-truth condition printed on them, purely so the PyBullet loop has
a real image file to load, decode, and pass into classify_pod() at each
stop -- exercising the same code path real Roboflow/Mendeley crops will
use later. Swap the files in sample_images/ (or repoint
pod_registry.PODS[i]["image_path"]) for the real datasets when wiring
in the trained models; nothing else in the pipeline needs to change.
"""

import os
from PIL import Image, ImageDraw, ImageFont

from pod_registry import PODS, IMAGE_DIR

CROP_BASE_COLOR = {
    "cabbage": (86, 152, 90),     # green
    "lettuce": (140, 190, 90),    # lighter green
    "mushroom": (196, 178, 148),  # tan
}
DISEASED_TINT = (150, 60, 40)  # brownish-red overlay hint


def _font(size):
    try:
        return ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size
        )
    except OSError:
        return ImageFont.load_default()


def make_placeholder(path, crop, stage, diseased, size=(320, 320)):
    base = CROP_BASE_COLOR[crop]
    if diseased:
        # blend toward the disease tint so diseased tiles are visually
        # distinguishable at a glance during the live demo
        base = tuple(int(b * 0.6 + t * 0.4) for b, t in zip(base, DISEASED_TINT))

    img = Image.new("RGB", size, base)
    draw = ImageDraw.Draw(img)

    # a few mottled patches so it doesn't read as a flat swatch
    import random
    rnd = random.Random(f"{crop}-{stage}-{diseased}")
    for _ in range(40):
        x, y = rnd.randint(0, size[0]), rnd.randint(0, size[1])
        r = rnd.randint(6, 22)
        shade = tuple(max(0, min(255, c + rnd.randint(-25, 25))) for c in base)
        draw.ellipse([x - r, y - r, x + r, y + r], fill=shade)

    if diseased:
        rnd2 = random.Random(f"spots-{crop}-{stage}")
        for _ in range(8):
            x, y = rnd2.randint(20, size[0] - 20), rnd2.randint(20, size[1] - 20)
            r = rnd2.randint(4, 10)
            draw.ellipse([x - r, y - r, x + r, y + r], fill=(40, 25, 15))

    label = f"{crop.upper()}\n{stage}\n{'DISEASED' if diseased else 'healthy'}"
    draw.rectangle([0, 0, size[0], 54], fill=(0, 0, 0))
    draw.multiline_text((8, 4), label, fill=(255, 255, 255), font=_font(14), spacing=2)

    img.save(path)


def generate_all():
    os.makedirs(IMAGE_DIR, exist_ok=True)
    written = set()
    for pod in PODS:
        path = pod["image_path"]
        if path in written:
            continue
        make_placeholder(
            path,
            crop=pod["crop_type"],
            stage=pod["ground_truth"]["growth_stage"],
            diseased=pod["ground_truth"]["diseased"],
        )
        written.add(path)
    print(f"Wrote {len(written)} placeholder images to {IMAGE_DIR}")


if __name__ == "__main__":
    generate_all()
