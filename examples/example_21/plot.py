"""
Utility script for plotting with matplotlib
"""
import matplotlib.pyplot as plt
import csv
import math
import numpy as np


def plot_values(time_vec, values):
    
	plt.plot(time_vec, values)
	plt.axhline(-0.37727, color='r', linestyle='--')
	plt.xlabel('time')
	plt.ylabel('Volt')
	plt.title('State')
	plt.show()

def main(filename):
	
	with open(filename, 'r', newline='') as csvfile:
		csv_file_reader = csv.reader(csvfile, delimiter=",")
		
		time = []
		value_func = []
		for row in csv_file_reader:
			if not row[0].startswith('#'):
				try:
					time.append(float(row[0]))
					value_func.append(float(row[1]))
				except:
					continue
		plot_values(time, value_func)
       
		
if __name__ == '__main__':
  main("/home/alex/qi3/cuberl/build/examples/filtering/filtering_example_1/state.csv")
	
	


