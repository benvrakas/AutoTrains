"""Draws the Auto Trains icon: two tracks crossing, one train passing through and one waiting short of the crossing,
under a crossed-out signal.

  python Tools/make_icon.py

Writes Docs/Images/icon-1080.png (full resolution), Docs/Images/icon-512.png (mod page) and Resources/Icon128.png
(shown in the game's Mods menu). The design is laid out on a 512-unit grid and rendered at full resolution.
"""
import os
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIZE = 512  # Design grid; all coordinates below are in these units.
FULL_SIZE = 1080
SUPERSAMPLE = 2  # Drawn larger than FULL_SIZE and scaled down for smooth edges.
SCALE = FULL_SIZE * SUPERSAMPLE / SIZE

BACKGROUND_TOP = (38, 44, 54)
BACKGROUND_BOTTOM = (22, 26, 32)
RAIL = (132, 138, 148)
SLEEPER = (82, 66, 52)
ORANGE = (250, 149, 73)
ORANGE_DARK = (196, 104, 40)
GREY = (104, 110, 120)
GREY_DARK = (70, 75, 84)
WINDOW = (36, 40, 46)
WHITE = (238, 240, 242)
RED = (214, 64, 56)
POLE = (150, 156, 166)


def s(value):
    return int(value * SCALE)


def box(x0, y0, x1, y1):
    return [s(x0), s(y0), s(x1), s(y1)]


def new_layer():
    big = FULL_SIZE * SUPERSAMPLE
    return Image.new("RGBA", (big, big), (0, 0, 0, 0))


def draw_track(draw, y):
    for sx in range(-40, SIZE + 40, 30):
        draw.rectangle(box(sx, y - 26, sx + 14, y + 26), fill=SLEEPER)
    draw.rectangle(box(-40, y - 20, SIZE + 40, y - 13), fill=RAIL)
    draw.rectangle(box(-40, y + 13, SIZE + 40, y + 20), fill=RAIL)


def draw_train(draw, x, y, cars, body, trim):
    """Draws a top-down train facing right whose nose is at x, centred on y."""
    car, gap, half = 70, 8, 24
    nose = x
    # Locomotive with a tapered nose.
    draw.polygon([(s(nose - car), s(y - half)), (s(nose - 16), s(y - half)), (s(nose), s(y - 10)), (s(nose), s(y + 10)),
                  (s(nose - 16), s(y + half)), (s(nose - car), s(y + half))], fill=body)
    draw.rectangle(box(nose - 30, y - 14, nose - 18, y + 14), fill=WINDOW)
    for index in range(cars):
        right = nose - car - gap - index * (car + gap)
        draw.rectangle(box(right + 2, y - 5, right + gap - 2, y + 5), fill=trim)
        draw.rounded_rectangle(box(right - car, y - half, right, y + half), radius=s(6), fill=body)
        draw.line([s(right - car + 10), s(y), s(right - 10), s(y)], fill=trim, width=s(5))


def draw_rotated(image, angle, painter):
    layer = new_layer()
    painter(ImageDraw.Draw(layer))
    rotated = layer.rotate(angle, resample=Image.BICUBIC, center=(image.width / 2, image.height / 2))
    image.alpha_composite(rotated)


def main():
    big = FULL_SIZE * SUPERSAMPLE
    image = Image.new("RGBA", (big, big))
    draw = ImageDraw.Draw(image)
    for row in range(big):
        t = row / big
        color = tuple(int(BACKGROUND_TOP[i] + (BACKGROUND_BOTTOM[i] - BACKGROUND_TOP[i]) * t) for i in range(3))
        draw.line([(0, row), (big, row)], fill=color)

    centre = SIZE / 2
    # The waiting train's track runs down-left to up-right; it stops short of the crossing.
    draw_rotated(image, 32, lambda d: (draw_track(d, centre), draw_train(d, centre - 92, centre, 2, GREY, GREY_DARK)))
    # The moving train's track runs up-left to down-right; it is through the middle of the crossing.
    draw_rotated(image, -32, lambda d: (draw_track(d, centre), draw_train(d, centre + 150, centre, 3, ORANGE, ORANGE_DARK)))

    # A crossed-out signal above the crossing.
    draw = ImageDraw.Draw(image)
    sx, sy = SIZE / 2, 92
    draw.rectangle(box(sx - 5, sy + 30, sx + 5, sy + 92), fill=POLE)
    draw.rounded_rectangle(box(sx - 24, sy - 42, sx + 24, sy + 34), radius=s(10), fill=GREY_DARK)
    draw.ellipse(box(sx - 15, sy - 32, sx + 15, sy - 2), fill=RED)
    draw.ellipse(box(sx - 15, sy, sx + 15, sy + 28), fill=WINDOW)
    draw.ellipse(box(sx - 58, sy - 58, sx + 58, sy + 58), outline=WHITE, width=s(9))
    draw.line([s(sx - 41), s(sy + 41), s(sx + 41), s(sy - 41)], fill=WHITE, width=s(9))

    full = image.convert("RGB").resize((FULL_SIZE, FULL_SIZE), Image.LANCZOS)
    docs = os.path.join(ROOT, "Docs", "Images")
    resources = os.path.join(ROOT, "Resources")
    os.makedirs(docs, exist_ok=True)
    os.makedirs(resources, exist_ok=True)
    full.save(os.path.join(docs, "icon-1080.png"))
    full.resize((512, 512), Image.LANCZOS).save(os.path.join(docs, "icon-512.png"))
    full.resize((128, 128), Image.LANCZOS).save(os.path.join(resources, "Icon128.png"))
    print("Wrote Docs/Images/icon-1080.png, Docs/Images/icon-512.png and Resources/Icon128.png")


if __name__ == "__main__":
    main()
