"""Make a GIF of multi-armed bandit methods learning which arm is best.

The bandit examples write pulls.csv: a line "# p,..." with the success
probability of every arm, then a row for every pull: method, pull, arm,
reward. This script draws, for every method, how often it has pulled every
arm so far, with the arm's success probability under it, and its mean reward
so far, every --every pulls.

Usage: python bandit_gif.py pulls.csv --out bandits.gif [--every 20]

Needs matplotlib and imageio.
"""
import argparse
from collections import OrderedDict

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read(path):
    p = None
    pulls = OrderedDict()
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# p"):
                p = [float(v) for v in line.split(",")[1:]]
                continue
            if line.startswith("#"):
                continue
            method, _, arm, reward = line.split(",")
            pulls.setdefault(method, []).append((int(arm), float(reward)))
    return p, pulls


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pulls")
    parser.add_argument("--out", required=True)
    parser.add_argument("--every", type=int, default=20)
    parser.add_argument("--fps", type=float, default=8)
    args = parser.parse_args()

    p, pulls = read(args.pulls)
    n_arms = len(p)
    n = max(len(v) for v in pulls.values())
    best = int(np.argmax(p))
    frames = []
    for t in list(range(args.every, n + 1, args.every)) + [n]:
        fig, axes = plt.subplots(1, len(pulls), figsize=(4.2 * len(pulls), 3.4), dpi=80, squeeze=False)
        for ax, (method, seq) in zip(axes[0], pulls.items()):
            seq = seq[:t]
            counts = np.bincount([a for a, _ in seq], minlength=n_arms)
            colours = ["#2e86ab" if a != best else "#e07a5f" for a in range(n_arms)]
            ax.bar(range(n_arms), counts, color=colours)
            ax.set_ylim(0, n)
            ax.set_xticks(range(n_arms))
            if n_arms <= 6:
                ax.set_xticklabels([f"arm {a}\np = {p[a]:.2f}" for a in range(n_arms)], fontsize=8)
            else:
                ax.set_xticklabels([f"{a}\n{p[a]:.2f}" for a in range(n_arms)], fontsize=7)
                ax.set_xlabel("arm, and its probability of success", fontsize=8)
            mean = np.mean([r for _, r in seq]) if seq else 0.0
            ax.set_title(f"{method}: {t} pulls, mean reward {mean:.2f}", fontsize=9)
            ax.set_ylabel("pulls", fontsize=8)
        fig.tight_layout()
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
        plt.close(fig)
    frames += [frames[-1]] * int(2 * args.fps)
    imageio.mimsave(args.out, frames, duration=1.0 / args.fps, loop=0)
    print(f"Wrote {args.out}: {len(frames)} frames")


if __name__ == "__main__":
    main()
