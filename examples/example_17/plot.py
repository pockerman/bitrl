"""
Utility script for plotting with matplotlib
"""
import matplotlib.pyplot as plt
import csv
import math
import numpy as np
from pathlib import Path


def running_mean(x, N=50):
    kernel = np.ones(N)
    conv_len = x.shape[0] - N
    y = np.zeros(conv_len)
    for i in range(conv_len):
        y[i] = kernel @ x[i:i + N]
        y[i] /= N
    return y


def plot_vals(filename: Path, title: str, N=100) -> None:
    with open(filename, 'r', newline='\n') as csvfile:

        csv_file_reader = csv.reader(csvfile, delimiter=",")

        rewards = []
        epochs = []
        for row in csv_file_reader:
            if not row[0].startswith('#'):
                epochs.append(int(row[0]))
                rewards.append(float(row[1]))

        avg_rewards = running_mean(np.array(rewards), N=N)
        plt.plot(avg_rewards)
        plt.title(title)
        plt.xlabel('Episode')

        if title == "Episode rewards":
            plt.ylabel('Reward')
        elif title == "Episode loss":
            plt.ylabel('Loss')
        elif title == "Episode duration":
            plt.ylabel('Number of iterations')
        plt.grid()
        plt.show()


if __name__ == '__main__':
    plot_vals("/home/alex/qi3/bitrl/build_cpu/examples/rl/rl_example_11/experiments/1/rewards.csv",
              "Episode rewards", N=100)
			  
    plot_vals("/home/alex/qi3/bitrl/build_cpu/examples/rl/rl_example_11/experiments/1/policy_loss.csv",
              "Policy loss", N=100)
			  
    plot_vals("/home/alex/qi3/bitrl/build_cpu/examples/rl/rl_example_11/experiments/1/critic_loss.csv",
              "Critic loss", N=100)
    plot_vals("/home/alex/qi3/bitrl/build_cpu/examples/rl/rl_example_11/experiments/1/episode_duration.csv",
              "Episode duration", N=100)
