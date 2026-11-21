"""Plot the runs of an experiment that log_experiment wrote.

Usage: python plot.py <experiment directory> [output.png]

Reads config.json for what was run, and every <run>/episodes.csv, and plots
the return of every episode, averaged over 20 episodes, one line per run.
"""
import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_returns(path: Path) -> list[float]:
    returns = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if row and not row[0].startswith("#"):
                returns.append(float(row[1]))
    return returns


def moving_average(values: list[float], n: int) -> list[float]:
    out = []
    for i in range(len(values)):
        window = values[max(0, i - n + 1): i + 1]
        out.append(sum(window) / len(window))
    return out


def main(directory: Path, output: Path) -> None:
    config = json.loads((directory / "config.json").read_text())
    fig, ax = plt.subplots(figsize=(7, 4))
    for run in sorted(p for p in directory.iterdir() if (p / "episodes.csv").exists()):
        returns = read_returns(run / "episodes.csv")
        ax.plot(moving_average(returns, 20), label=run.name.replace("eta_", "eta = ").replace("p", "."))
    ax.set_xlabel("episode")
    ax.set_ylabel("return, mean of 20 episodes")
    ax.set_title(f"{config['algorithm']} on the {config['environment']}")
    ax.set_ylim(-150, 0)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=100)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2] if len(sys.argv) > 2 else "returns.png"))
