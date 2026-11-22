"""Make a GIF of an episode of bitrl's multi-agent examples.

grid:   multi_agent_grid_world writes layout.txt, the map ('#' a wall, a lower
        case letter the goal of the agent with that letter in upper case), and
        episode.csv, a row per agent per step: step,agent,row,col,done.
robots: multi_robot_navigation writes trajectories.csv, a row per robot per
        step: step,robot,x,y,theta,goal_x,goal_y,radius.

Usage: python multi_agent_gif.py grid layout.txt episode.csv --out grid.gif
       python multi_agent_gif.py robots trajectories.csv --out robots.gif [--every 2]

Needs matplotlib and imageio.
"""
import argparse
import csv
import math

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLOURS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b",
           "#e377c2", "#17becf", "#bcbd22", "#7f7f7f", "#393b79", "#637939"]


def frame(fig):
    fig.canvas.draw()
    image = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return image


def grid(layout_path, episode_path, out, title):
    layout = [line.rstrip("\n") for line in open(layout_path) if line.strip()]
    steps = {}
    with open(episode_path) as f:
        for row in csv.DictReader(f):
            steps.setdefault(int(row["step"]), []).append(
                (row["agent"], int(row["row"]), int(row["col"]), row["done"] == "1"))
    n_rows, n_cols = len(layout), len(layout[0])
    frames = []
    for k in sorted(steps):
        fig, ax = plt.subplots(figsize=(n_cols * 0.6, n_rows * 0.6 + 0.6), dpi=80)
        for r, line in enumerate(layout):
            for c, ch in enumerate(line):
                colour = "#444444" if ch == "#" else "#f7f7f7"
                ax.add_patch(plt.Rectangle((c, n_rows - 1 - r), 1, 1, color=colour))
                if ch.islower():
                    i = ord(ch) - ord("a")
                    ax.add_patch(plt.Rectangle((c + 0.1, n_rows - 0.9 - r), 0.8, 0.8, fill=False,
                                               lw=2, ec=COLOURS[i % len(COLOURS)]))
        for agent, r, c, done in steps[k]:
            i = ord(agent) - ord("A")
            ax.add_patch(plt.Circle((c + 0.5, n_rows - 0.5 - r), 0.35, color=COLOURS[i % len(COLOURS)],
                                    alpha=0.5 if done else 1.0))
            ax.text(c + 0.5, n_rows - 0.5 - r, agent, ha="center", va="center", color="white",
                    fontsize=11, weight="bold")
        ax.set_xlim(0, n_cols)
        ax.set_ylim(0, n_rows)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(f"{title}   step {k}", fontsize=10)
        frames.append(frame(fig))
    # hold the last picture
    frames += [frames[-1]] * 4
    imageio.mimsave(out, frames, duration=0.4, loop=0)


def robots(path, out, title, every):
    steps = {}
    goals = {}
    radius = 0.15
    with open(path) as f:
        for row in csv.DictReader(f):
            i = int(row["robot"])
            steps.setdefault(int(row["step"]), {})[i] = (float(row["x"]), float(row["y"]),
                                                       float(row["theta"]))
            goals[i] = (float(row["goal_x"]), float(row["goal_y"]))
            radius = float(row["radius"])
    keys = sorted(steps)
    extent = max(max(abs(g[0]), abs(g[1])) for g in goals.values()) + 0.5
    trails = {i: [] for i in goals}
    frames = []
    for k in keys:
        for i, (x, y, _) in steps[k].items():
            trails[i].append((x, y))
        if k % every and k != keys[-1]:
            continue
        fig, ax = plt.subplots(figsize=(4.5, 4.9), dpi=80)
        for i, (gx, gy) in goals.items():
            colour = COLOURS[i % len(COLOURS)]
            ax.plot(gx, gy, "x", color=colour, ms=8, mew=2)
            xs, ys = zip(*trails[i])
            ax.plot(xs, ys, "-", color=colour, lw=1, alpha=0.5)
            x, y, theta = steps[k][i]
            ax.add_patch(plt.Circle((x, y), radius, color=colour))
            ax.plot([x, x + radius * math.cos(theta)], [y, y + radius * math.sin(theta)], "w-", lw=2)
        ax.set_xlim(-extent, extent)
        ax.set_ylim(-extent, extent)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f"{title}   t = {k * 0.1:.1f} s", fontsize=10)
        frames.append(frame(fig))
    frames += [frames[-1]] * 10
    imageio.mimsave(out, frames, duration=0.1, loop=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="kind", required=True)
    g = sub.add_parser("grid")
    g.add_argument("layout")
    g.add_argument("episode")
    g.add_argument("--out", default="multi_agent_grid_world.gif")
    g.add_argument("--title", default="Independent Q-learning")
    r = sub.add_parser("robots")
    r.add_argument("trajectories")
    r.add_argument("--out", default="multi_robot_navigation.gif")
    r.add_argument("--title", default="Decentralised robots")
    r.add_argument("--every", type=int, default=2, help="draw every so many steps")
    args = parser.parse_args()
    if args.kind == "grid":
        grid(args.layout, args.episode, args.out, args.title)
    else:
        robots(args.trajectories, args.out, args.title, args.every)


if __name__ == "__main__":
    main()
