"""
Utility script for plotting with matplotlib
"""
import matplotlib.pyplot as plt
import csv
import math
import numpy as np

def main(filename):
	
	with open(filename, 'r', newline='') as csvfile:
		csv_file_reader = csv.reader(csvfile, delimiter=",")
		
		time = []
		x_position_true = []
		y_position_true = []
		orientation_true = []
		x_position = []
		y_position = []
		orientation = []
		for row in csv_file_reader:
			if not row[0].startswith('#'):
				try:
					time.append(float(row[0]))
					x_position_true.append(float(row[1]))
					y_position_true.append(float(row[2]))
					orientation_true.append(float(row[3]))
					x_position.append(float(row[4]))
					y_position.append(float(row[5]))
					orientation.append(float(row[6]))
				except:
					continue
		#plot_values(time, value_func)

		plt.plot(time, x_position_true)
		plt.plot(time, x_position)
		
		plt.xlabel('time')
		plt.ylabel('X position')
		plt.title('Comparison of X-coordinate')
		plt.show()

		plt.plot(time, y_position_true)
		plt.plot(time, y_position)
		
		plt.xlabel('time')
		plt.ylabel('Y position')
		plt.title('Comparison of Y-coordinate')
		plt.show()

		plt.plot(time, orientation_true)
		plt.plot(time, orientation)
		
		plt.xlabel('time')
		plt.ylabel('Orientation')
		plt.title('Comparison of Orientation')
		plt.show()
       
		
if __name__ == '__main__':
  main("/home/alex/qi3/cuberl/build/examples/filtering/filtering_example_2/state.csv")
	
	


