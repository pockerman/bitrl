"""
Utility script for plotting with matplotlib
"""
import matplotlib.pyplot as plt
import csv
import math
import numpy as np
from pathlib import Path


def plot_rewards(filename: Path) -> None:


    with open(filename, 'r', newline='\n') as csvfile:

        csv_file_reader = csv.reader(csvfile, delimiter=",")
		
        rewards = []
        epochs = []
        for row in csv_file_reader:
            if not row[0].startswith('#'):
                try:
                    epochs.append(int(row[0]))
                    rewards.append(float(row[1]))
                except:
                    continue

        plt.plot(epochs, rewards)
        plt.title('Average reward per episode')
        plt.xlabel('Epoch')
        plt.ylabel('Reward')
        plt.grid()
        plt.show()

		
if __name__ == '__main__':
  plot_rewards("/home/alex/qi3/bitrl/build/examples/rl/rl_example_10/reward_per_itr.csv")
	
	


