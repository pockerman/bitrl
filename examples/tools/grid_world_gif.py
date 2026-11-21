"""Make a GIF of episodes on bitrl's Gridworld.

The DQN examples write episode.csv after training: a row for every step, the
episode, the action that led to the board (-1 for the first board), the reward
and the board, 4 x side x side values: a one-hot plane each for the player,
the goal, the pit and the wall, row by row.

Usage: python grid_world_gif.py episode.csv --out dqn.gif [--title DQN] [--side 4] [--episodes 1,2,0]

Needs matplotlib and imageio.
"""
import argparse

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ACTIONS = {-1: "start", 0: "up", 1: "down", 2: "left", 3: "right"}
PIECES = [("goal", 1, "#f4c542"), ("pit", 2, "#d1495b"), ("wall", 3, "#555555")]


def read(path, side):
    episodes = {}
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            v = line.strip().split(",")
            board = np.array([float(x) for x in v[3:]]).reshape(4, side, side)
            episodes.setdefault(int(v[0]), []).append((int(v[1]), float(v[2]), board))
    return episodes


def where(plane):
    r, c = np.unravel_index(np.argmax(plane), plane.shape)
    return r, c


def draw(board, side, title, trail):
    fig, ax = plt.subplots(figsize=(3.6, 3.9), dpi=80)
    ax.set_xlim(-0.5, side - 0.5)
    ax.set_ylim(side - 0.5, -0.5)
    for k in range(side + 1):
        ax.plot([k - 0.5, k - 0.5], [-0.5, side - 0.5], color="#bbbbbb", lw=1)
        ax.plot([-0.5, side - 0.5], [k - 0.5, k - 0.5], color="#bbbbbb", lw=1)
    for name, idx, colour in PIECES:
        if board[idx].max() > 0.5:
            r, c = where(board[idx])
            ax.add_patch(plt.Rectangle((c - 0.45, r - 0.45), 0.9, 0.9, color=colour))
            ax.text(c, r + 0.32, name, ha="center", va="center", fontsize=8,
                    color="white" if name != "goal" else "black")
    if len(trail) > 1:
        ys, xs = zip(*trail)
        ax.plot(xs, ys, color="#2e86ab", lw=2, alpha=0.5)
    if board[0].max() > 0.5:
        r, c = where(board[0])
        ax.add_patch(plt.Circle((c, r - 0.05), 0.25, color="#2e86ab"))
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.canvas.draw()
    frame = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
    plt.close(fig)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv")
    parser.add_argument("--out", required=True)
    parser.add_argument("--title", default="")
    parser.add_argument("--side", type=int, default=4)
    parser.add_argument("--max-episodes", type=int, default=5)
    parser.add_argument("--episodes", help="the episodes to show, in order, as 1,2,0; default the first ones")
    parser.add_argument("--fps", type=float, default=3)
    args = parser.parse_args()

    frames = []
    episodes = read(args.csv, args.side)
    chosen = [int(e) for e in args.episodes.split(",")] if args.episodes else sorted(episodes)[: args.max_episodes]
    for e in chosen:
        steps = episodes[e]
        ret = sum(r for a, r, _ in steps[1:])
        trail = []
        for i, (action, reward, board) in enumerate(steps):
            if board[0].max() > 0.5:
                trail.append(where(board[0]))
            label = f"{args.title}, board {e + 1}: " if len(episodes) > 1 else f"{args.title}: "
            label += f"step {i}, {ACTIONS[action]}"
            if i == len(steps) - 1:
                label += f", return {ret:g}"
            frames.append(draw(board, args.side, label, trail))
        frames += [frames[-1]] * int(args.fps)
    imageio.mimsave(args.out, frames, duration=1.0 / args.fps, loop=0)
    print(f"Wrote {args.out}: {len(frames)} frames")


if __name__ == "__main__":
    main()
