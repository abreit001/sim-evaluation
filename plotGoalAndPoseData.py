#!/usr/bin/env python3

DEBUG = 1
import sys
import csv
import scipy
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plot
from mpl_toolkits.axes_grid.axislines import SubplotZero
import matplotlib.colors as mcolors
import random

data1       = []
successIndices = []
failureIndices = []
data1X      = []
data1Y      = []
failures    = []
successes   = []
poseTuple1  = []
allTrials   = []

goalFile = 'goalData.csv'
poseFile = 'poseData.csv'

# 1a.) Organize pose data by appending tuples of (x,y) coordinates into a trek
#	   represented as a list. Upon completing the trek, append it into a list
#	   of treks (ie. a list of lists):
with open(Path.cwd().joinpath(poseFile), newline='') as csv1:
	# print('opened file\n')
	fileReader = csv.reader(csv1)
	# 2b.) Collect all the relevant data:
	currentList = []
	for line in fileReader:
		if line[0] == '%time':
			continue
		elif line[0] == '111111111':
			poseTuple1.append(currentList)
			currentList = []
		else:
			currentList.append((round(float(line[1]), 4), round(float(line[2]), 4)))
	poseTuple1.append(currentList)

if DEBUG:
	print('\nposeTuple1 size: {}'.format(len(poseTuple1)))

emptyTrials = []
# Find any empty pose data
for n in range(len(poseTuple1)):
	if len(poseTuple1[n]) == 0:
		emptyTrials.append(n)

#TODO: Turn these into functions for importing
# 2a.) Get the goal data:
with open(Path.cwd().joinpath(goalFile), newline='') as csv1:
	# print('opened file\n')
	fileReader = csv.reader(csv1)
	
	# 2b.) Collect all the relevant data:
	for line in fileReader:
		if line[0] == '%time' or line[0] == '111111':
			continue
		else:
			data1.append(float(line[1]))
	if DEBUG:
		print('data1 size: {}'.format(len(data1)))
	
	# 2c.) Distribute the data according to whether or not the tuple at data[n]
	#     has a corresponing 'goalReached' tuple. If it does, put the coordinate
	#     in the 'successes' list:
	n=0
	loopCount = 0
	while n < len(data1)-1:
		# try:
		# 	poseX,poseY = zip(*poseTuple1[loopCount])
		# except:
		# 	pass
		# print('\nloopCount: ' + str(loopCount))
		# print('n:         ' + str(n))
		# print(data1[n])
		# print(data1[n+3])
		# print(poseX[-1])
		# print(data1[n+1])
		# print(-1*data1[n+2])
		# print(poseY[-1])
		allTrials.append((data1[n], data1[n+1]))
		# Skip over this index if we did not find data for this trial
		if loopCount in emptyTrials:
			loopCount = loopCount + 1
		# If the goal is close enough to the final point of the rover,
		# consider the trial a success (previously elif data1[n] == data1[n + 3] and data1[n+1] == -1*data1[n+2]:)
		else:
			try:
				if np.abs(data1[n] - data1[n + 3]) < 0.001 and np.abs(data1[n+1] + data1[n+2]) < 0.001:
					successes.append((data1[n], data1[n+1]))
					successIndices.append(loopCount)
					n  = n + 4
					loopCount = loopCount + 1
				else:
					# 2d.) If there is no corresponding 'goalReached' tuple, put the
					#      data point in the 'failures' list:
					failures.append((data1[n], data1[n+1]))
					failureIndices.append(loopCount)
					n  = n + 2
					loopCount = loopCount + 1
			except:
				# 2d.) If there is no corresponding 'goalReached' tuple, put the
				#      data point in the 'failures' list:
				failures.append((data1[n], data1[n+1]))
				failureIndices.append(loopCount)
				n  = n + 2
				loopCount = loopCount + 1
	if DEBUG:
		print('loop count: ' + str(loopCount))

print('\nTrials without data: ' + ', '.join('{}'.format(s) for s in emptyTrials))
print('\nTotal Successes: {}'.format(len(successIndices)))
print('Success Indices:\n' + ', '.join('{}'.format(s) for s in successIndices))
print('\nTotal Failures: {}'.format(len(failureIndices)))
print('Failure Indices:\n' + ', '.join('{}'.format(s) for s in failureIndices))

# Plotting the data:

fig, ax = plot.subplots(figsize=(10,10))
# 1.) Separate the coordinate tuples into lists of x and y coordinates:
x,y = zip(*successes)
xf,yf = zip(*failures)
x_all, y_all = zip(*allTrials)

# 2.) Transform from x-forward to y-forward for visualization purposes, and translate
#     according to the origin x = 8, y = 5:
origin_x = 0
origin_y = 0
xn    = [(i * -1) - origin_y for i in y]
xfn   = [(i * -1) - origin_y for i in yf]
xalln = [(i * -1) - origin_y for i in y_all]
yn    = [(i + origin_x)	for i in x]
yfn   = [(i + origin_x) for i in xf]
yalln = [(i + origin_x) for i in x_all]


# Separate each trek's pose tuples into lists of x and y coordinates.
# Transform them from x-forward to y-forward for visualization purposes.
# Translate them according to the origin x = 8, y = 5. Plot them:

counter = 0

PLOT_ALL = 0
PLOT_SUCCESS = 1
PLOT_FAILURE = 0

if PLOT_ALL:
	# Plot all trials
	# Colors of goals and treks should match -- if not, something is wrong
	colors = list(mcolors.CSS4_COLORS)
	num_colors = len(colors)
	for n in range(len(poseTuple1)):
		try:
			poseX,poseY = zip(*poseTuple1[n])
		except:
			continue
		poseXn = [((-1*y) - origin_y) for y in poseY]
		poseYn = [(t+origin_x) for t in poseX]
		ax.plot(poseXn, poseYn, str(colors[n%num_colors]))
		# if (np.abs(poseXn[-1]-xalln[n]) > 2) or (np.abs(poseYn[-1]-yalln[n]) > 2):
		# 	print('\nIndex: {}'.format(n))
		# 	print('Final pose:')
		# 	print(poseXn[-1])
		# 	print(poseYn[-1])
		# 	print('Goal:')
		# 	print(xalln[n])
		# 	print(yalln[n])
		try:
			ax.scatter(xalln[n], yalln[n], c=colors[n%num_colors], zorder=1)
		except:
			pass
		counter = counter + 1
	ax.scatter(xfn, yfn, c='r', marker='x', zorder=1)
	print('\nPlotted all trials')

if PLOT_SUCCESS:
	# Plot all successful trials
	for n in successIndices:
		poseX,poseY = zip(*poseTuple1[n])
		poseXn = [((-1*y) - origin_y) for y in poseY]
		poseYn = [(t+origin_x) for t in poseX]
		ax.plot(poseXn, poseYn, 'mediumseagreen')
		ax.scatter(xalln[n], yalln[n], c='g', zorder=1)
		counter = counter + 1
	print('\nPlotted successful trials')

if PLOT_FAILURE:
	colors = list(mcolors.TABLEAU_COLORS)
	num_colors = len(colors)
	# Plot all failed trials
	for n in failureIndices:
		poseX,poseY = zip(*poseTuple1[n])
		poseXn = [((-1*y) - origin_y) for y in poseY]
		poseYn = [(t+origin_x) for t in poseX]
		ax.plot(poseXn, poseYn, colors[counter%num_colors])
		ax.scatter(xalln[n], yalln[n], c=colors[counter%num_colors], marker='x', zorder=1)
		counter = counter + 1
	ax.legend(failureIndices, title='Trial Number')
	print('\nPlotted failed trials')

if DEBUG:
	print('\npose plot counter: {}'.format(counter))

# 3.) Specify the axes of the plot:
# xmin, ymin, xmax, ymax = -50, -50, 50, 50
size = 129/2
xmin, ymin, xmax, ymax = -size, -size, size, size
ax.set(xlim=(xmin-1, xmax+1), ylim=(ymin-1, ymax+1), aspect='equal')
ticks_freq = 1

# 5.) Set bottom and left spines as x and y axes of coordinate system
ax.spines['bottom'].set_position('zero')
ax.spines['left'].set_position('zero')

# 6.) Remove top and right spines:
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# 7.) Draw grid lines:
ax.grid(which='major', color='grey', linewidth=1, linestyle='-', alpha=0.2)
ax.grid(which='minor', color='grey', linewidth=1, linestyle='-', alpha=0.2)

# 8.) Import image of terrain for added insight:
img = plot.imread(Path.cwd().joinpath('terrain_map.jpeg'))
ax.imshow(img, zorder=0, extent=[xmin, xmax, ymin, ymax])

plot.show()

# if __name__ == "__main__":
# 	main()