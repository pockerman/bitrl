import matplotlib.pyplot as plt
import csv
from pathlib import Path


if __name__ == '__main__':

	filename = Path('/home/alex/qi3/bitrl/build_cpu/examples/rl/rl_example_12/experiments/3/dqn_grid_world_policy_losses.csv')
	
	with open(filename, 'r') as f:
		reader = csv.reader(f, delimiter=',')
		
		values = []
		
		for line in reader:
			values.append(float(line[0]))
			
		
		
		plt.figure(figsize=(10,7))
		plt.plot(values)
		plt.xlabel("Epochs",fontsize=22)
		plt.ylabel("Average epoch loss",fontsize=22)
		plt.show()
