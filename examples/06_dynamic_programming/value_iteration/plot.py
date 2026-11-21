"""
Utility script for plotting with matplotlib
"""
import matplotlib.pyplot as plt
import csv
import math
import numpy as np
from pathlib import Path
import sys


def plot_state_values(v, title:str) -> None:
    # reshape value function
    V_sq = np.reshape(v, (4, 4))

    # plot the state-value function
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    im = ax.imshow(V_sq, cmap='cool')
    for (j, i), label in np.ndenumerate(V_sq):
        ax.text(i, j, np.round(label, 5), ha='center', va='center', fontsize=14)
    plt.tick_params(bottom=False, left=False, labelbottom=False, labelleft=False)
    plt.title(title)
    plt.show()

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
        plt.title('$\Delta$ value per epoch')
        plt.grid()
        plt.show()



def main(filename):
	
	with open(filename, 'r', newline='') as csvfile:
		csv_file_reader = csv.reader(csvfile, delimiter=",")
		
		value_func = []
		for row in csv_file_reader:
			if not row[0].startswith('#'):
				try:
					value_func.append(float(row[1]))
				except:
					continue
		plot_state_values(np.array(value_func), title='State value function')
       
		
if __name__ == '__main__':
    # Run it from the directory the example wrote its files to, or pass that directory
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('.')
    main(out_dir / "value_iteration_frozen_lake_v1.csv")
    plot_rewards(out_dir / "reward_per_itr.csv")
