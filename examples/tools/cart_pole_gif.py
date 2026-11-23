"""Make a GIF of a cart-pole episode that an example wrote.

es_cart_pole_swing_up writes swing_up.csv, a row per step: t,x,theta,force,
with theta the angle of the pole from upright [rad] and force the push on the
cart [N]. The GIF shows the cart on its track, the pole, and the force as an
arrow, in real time.

Usage: python cart_pole_gif.py swing_up.csv --out swing_up.gif [--title "..."] [--every 2]

Needs imageio and Pillow.
"""
import argparse
import csv
import math

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

WIDTH, HEIGHT = 640, 300
SCALE = 100.0                 # pixels per metre
TRACK = 2.4                   # half the length of the track [m]
POLE = 1.0                    # the length of the pole [m]
GROUND = 210                  # the height of the track in the picture [px]
MAX_FORCE = 10.0


def draw(row, title):
    image = Image.new("RGB", (WIDTH, HEIGHT), "white")
    d = ImageDraw.Draw(image)
    cx = WIDTH / 2
    # the track and its ends
    d.line([(cx - TRACK * SCALE, GROUND), (cx + TRACK * SCALE, GROUND)], fill="#555555", width=3)
    for end in (-TRACK, TRACK):
        x = cx + end * SCALE
        d.line([(x, GROUND - 12), (x, GROUND + 12)], fill="#d62728", width=3)
    # the cart
    x = cx + row["x"] * SCALE
    d.rectangle([x - 30, GROUND - 18, x + 30, GROUND + 2], fill="#1f77b4")
    # the pole, from the pivot on top of the cart
    theta = row["theta"]
    px, py = x, GROUND - 18
    tx = px + POLE * SCALE * math.sin(theta)
    ty = py - POLE * SCALE * math.cos(theta)
    d.line([(px, py), (tx, ty)], fill="#ff7f0e", width=7)
    d.ellipse([px - 4, py - 4, px + 4, py + 4], fill="black")
    # the force, as an arrow under the cart
    f = row["force"] / MAX_FORCE * 60
    if abs(f) > 1:
        y = GROUND + 30
        d.line([(x, y), (x + f, y)], fill="#2ca02c", width=4)
        s = 1 if f > 0 else -1
        d.polygon([(x + f, y), (x + f - s * 8, y - 6), (x + f - s * 8, y + 6)], fill="#2ca02c")
    up = abs(math.atan2(math.sin(theta), math.cos(theta))) < math.radians(12)
    d.text((10, 10), title, fill="black")
    d.text((10, 28), f"t = {row['t']:.2f} s   force = {row['force']:+.1f} N", fill="black")
    d.text((10, 46), "upright" if up else "", fill="#2ca02c")
    return np.asarray(image)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv")
    parser.add_argument("--out", default="cart_pole.gif")
    parser.add_argument("--title", default="Evolution strategies: swing up and balance")
    parser.add_argument("--every", type=int, default=2, help="draw every so many steps")
    args = parser.parse_args()

    with open(args.csv) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    rows = rows[:: args.every]
    dt = rows[1]["t"] - rows[0]["t"] if len(rows) > 1 else 0.04
    frames = [draw(r, args.title) for r in rows]
    imageio.mimsave(args.out, frames, duration=dt, loop=0)
    print(f"{args.out}: {len(frames)} frames")


if __name__ == "__main__":
    main()
