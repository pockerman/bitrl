"""Make a GIF of a value function converging on FrozenLake.

The dynamic programming examples write sweeps.csv: a row for every sweep, the
sweep and then the value of every state. This script draws every sweep as the
lake, coloured by the value of its cells, with the value written in each cell
and an arrow for the greedy action, the one with the largest expected
r + gamma V(s') under FrozenLake's own transition model.

Usage:
  python value_gif.py sweeps.csv --out value_iteration.gif --title "Value iteration" [--slippery] [--gamma 0.99]
      [--step iteration]

Needs gymnasium, matplotlib and imageio.
"""
import argparse

import gymnasium as gym
import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# FrozenLake's actions: 0 left, 1 down, 2 right, 3 up
ARROWS = {0: (-0.3, 0), 1: (0, 0.3), 2: (0.3, 0), 3: (0, -0.3)}


def read_sweeps(path):
    sweeps = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                values = [float(v) for v in line.split(",")]
                sweeps.append((int(values[0]), np.array(values[1:])))
    return sweeps


def greedy_actions(P, V, gamma):
    actions = []
    for s in range(len(V)):
        q = [sum(p * (r + gamma * V[s2] * (not d)) for p, s2, r, d in P[s][a]) for a in range(4)]
        actions.append(int(np.argmax(q)))
    return actions


def draw(desc, V, actions, title, vmax):
    n = desc.shape[0]
    fig, ax = plt.subplots(figsize=(4, 4.4), dpi=80)
    grid = V.reshape(n, n)
    ax.imshow(grid, cmap="Blues", vmin=0, vmax=vmax)
    for r in range(n):
        for c in range(n):
            s = r * n + c
            cell = desc[r, c].decode()
            if cell == "H":
                ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color="#333333"))
                ax.text(c, r, "hole", ha="center", va="center", color="white", fontsize=8)
                continue
            if cell == "G":
                ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color="#f4c542"))
                ax.text(c, r, "goal", ha="center", va="center", fontsize=8)
                continue
            dark = grid[r, c] > 0.6 * vmax
            ax.text(c, r + 0.3, f"{grid[r, c]:.3f}", ha="center", va="center", fontsize=7,
                    color="white" if dark else "black")
            dx, dy = ARROWS[actions[s]]
            ax.arrow(c - dx / 2, r - dy / 2 - 0.08, dx, dy, head_width=0.12, head_length=0.1,
                     color="#d1495b", length_includes_head=True)
            if cell == "S":
                ax.text(c - 0.38, r - 0.36, "S", fontsize=8, fontweight="bold")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.canvas.draw()
    frame = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
    plt.close(fig)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sweeps")
    parser.add_argument("--out", required=True)
    parser.add_argument("--title", default="")
    parser.add_argument("--slippery", action="store_true")
    parser.add_argument("--gamma", type=float, default=1.0)
    parser.add_argument("--map", default="4x4")
    parser.add_argument("--max-frames", type=int, default=40, help="sweeps to show at most, spread evenly")
    parser.add_argument("--fps", type=float, default=4)
    parser.add_argument("--step", default="sweep", help="what a row of the file is, for the titles")
    args = parser.parse_args()

    env = gym.make("FrozenLake-v1", is_slippery=args.slippery, map_name=args.map).unwrapped
    sweeps = read_sweeps(args.sweeps)
    if len(sweeps) > args.max_frames:
        keep = np.unique(np.round(np.geomspace(1, len(sweeps), args.max_frames)).astype(int) - 1)
        sweeps = [sweeps[i] for i in keep]
    vmax = max(1e-9, max(V.max() for _, V in sweeps))
    frames = [draw(env.desc, V, greedy_actions(env.P, V, args.gamma), f"{args.title}: {args.step} {k + 1}", vmax)
              for k, V in sweeps]
    frames += [frames[-1]] * int(2 * args.fps)
    imageio.mimsave(args.out, frames, duration=1.0 / args.fps, loop=0)
    print(f"Wrote {args.out}: {len(sweeps)} sweeps shown")


if __name__ == "__main__":
    main()
