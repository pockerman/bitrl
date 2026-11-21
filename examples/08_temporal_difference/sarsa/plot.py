"""
Utility script for plotting with matplotlib
"""
import matplotlib.pyplot as plt
import csv
import math
import numpy as np


def plot_values(V):

    # reshape the state-value function
	V = np.reshape(V, (4,12))
	# plot the state-value function
	fig = plt.figure(figsize=(15,5))
	ax = fig.add_subplot(111)
	im = ax.imshow(V, cmap='cool')
	for (j,i),label in np.ndenumerate(V):
	    ax.text(i, j, np.round(label,3), ha='center', va='center', fontsize=14)
	plt.tick_params(bottom='off', left='off', labelbottom='off', labelleft='off')
	plt.title('State-Value Function')
	plt.show()
	
	
def main(v_func_filename, avg_score_filename):
	
	with open(v_func_filename, 'r', newline='') as csvfile:
		csv_file_reader = csv.reader(csvfile, delimiter=",")
		
		value_func = []
		for row in csv_file_reader:
			if not row[0].startswith('#'):
				try:
					value_func.append(float(row[1]))
				except:
					continue
		plot_values(np.array(value_func))
		
	with open(avg_score_filename, 'r', newline='') as csvfile:
		csv_file_reader = csv.reader(csvfile, delimiter=",")
		
		episode_id = []
		value_func = []
		for row in csv_file_reader:
			if not row[0].startswith('#'):
				try:
					episode_id.append(int(row[0]))
					value_func.append(float(row[1]))
				except:
					continue
		# plot performance
		plt.plot(episode_id, value_func)
		plt.xlabel('Episode Number')
		plt.ylabel('Average Reward (Over Next %d Episodes)' % 100)
		plt.show()
		
	
	
       
		
if __name__ == '__main__':
  main("/home/alex/qi3/bitrl/build/examples/rl/rl_example_9/sarsa_cliff_walking_v0.csv", 
       "/home/alex/qi3/bitrl/build/examples/rl/rl_example_9/reward_per_itr.csv")
	
	


