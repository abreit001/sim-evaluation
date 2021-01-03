DEBUG = 1
import sys
import os
import csv
import scipy
import numpy as np
import random
from pathlib import Path
import matplotlib.pyplot as plot
import matplotlib.colors as mcolors
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

# Initialize arrays that will hold the indices of the successes and failures
successIndices = []
failureIndices = []
# Initialize dictionaries that will hold the goals and poses for each trial
goals = {}
poses = {}

goalFile = 'goalDataLabeled.csv'
poseFile = 'poseDataLabeled.csv'

# Organize pose data by appending tuples of (x,y) coordinates into a path
# represented as a list. Upon completing the path, add it into a dictionary
# of poses (a dictionary of lists)
def readPoseFile():
    with open(os.getcwd()+'/'+poseFile) as pose_csv:
        
        print('Opened pose file')
        fileReader = csv.reader(pose_csv)
        
        currentList = []
        trial_num = 0

        for line in fileReader:
            # Assume we start at trial number 0
            if line[0] == 'Trial 0':
                continue
            
            # Skip over this line, which is just a header at the start of each trial
            elif line[0] == '%time':
                continue
            
            # When we get to the start of another trial, add the current list
            # to the poses dictionary and update the trial number
            elif line[0][0:5] == 'Trial':
                poses[trial_num] = currentList
                currentList = []
                trial_num = int(line[0][5:])
            
            # Otherwise, add the line as a pose tuple to the current list
            else:
                currentList.append((round(float(line[1]), 4), round(float(line[2]), 4)))
        
        # Add the last path to the dictionary
        poses[trial_num] = currentList

# Organize pose data by appending all pose data coordinates into a path
# represented as a list. Upon completing the path, add it into a dictionary
# of poses (a dictionary of lists)
def readPoseFileVerbose():
    with open(os.getcwd()+'/'+poseFile) as pose_csv:
        
        print('Opened pose file')
        fileReader = csv.reader(pose_csv)
        
        currentList = []
        trial_num = 0

        for line in fileReader:
            # Assume we start at trial number 0
            if line[0] == 'Trial 0':
                continue
            
            # Skip over this line, which is just a header at the start of each trial
            elif line[0] == '%time':
                continue
            
            # When we get to the start of another trial, add the current list
            # to the poses dictionary and update the trial number
            elif line[0][0:5] == 'Trial':
                poses[trial_num] = currentList
                currentList = []
                trial_num = int(line[0][5:])
            
            # Otherwise, add the line as a pose tuple to the current list
            else:
                time = line[0]
                pos = line[1:4]
                quat = line[4:8]
                euler = euler_from_quaternion(quat)
                currentList.append((time, pos, quat, euler))
        
        # Add the last path to the dictionary
        poses[trial_num] = currentList

# Get the goal data for each trial and determine whether each was
# a success or a failure
def readGoalFile():
    with open(os.getcwd()+'/'+goalFile) as goal_csv:
        
        print('Opened goal file')
        fileReader = csv.reader(goal_csv)
        
        tempGoals = {}
        currentList = []
        trial_num = 0
        for line in fileReader:
            # Assume we start at trial number 0
            if line[0] == 'Trial 0':
                continue
            
            # Skip over this line, which is just a header at the start of each trial
            elif line[0] == '%time':
                continue
            
            # When we get to the start of another trial, add the current list
            # to the tempGoals dictionary and update the trial number
            elif line[0][0:5] == 'Trial':
                tempGoals[trial_num] = currentList
                currentList = []
                trial_num = int(line[0][5:])
            
            # Otherwise, add the line to the current list
            else:
                currentList.append(float(line[1]))
        
        # Add the last goal data to the dictionary
        tempGoals[trial_num] = currentList
        
    # Distribute the data according to whether or not each trial has a
    # corresponding 'goalReached' tuple. If it does, it is a success.
    # In other words, failed trials will just have two values, the x and y
    # position of the goal. Successful trials will have four values, the
    # x and y position of the goal, and the x and y position of the goal reached
    for num, currentList in tempGoals.items():
        
        # If there is no data, just add an empty list to the goals dictionary
        if len(currentList) == 0:
            goals[num] = currentList
        
        # Failed trials
        elif len(currentList) == 2:
            goals[num] = currentList
            failureIndices.append(num)

        # Potentially successful trials
        elif len(currentList) == 4:
            goals[num] = currentList[0:2]
            # Check if the goal reached and the actual goal are close enough
            if np.abs(currentList[0] - currentList[3]) < 0.001 and np.abs(currentList[1] + currentList[2]) < 0.001:
                successIndices.append(num)
            else:
                failureIndices.append(num)

def findEmptyTrials():
    emptyPoses = []
    emptyGoals = []

    # Find any empty path data
    for n in range(len(poses)):
        if len(poses[n]) == 0:
            emptyPoses.append(n)

    # Find any empty goal data
    for n in range(len(goals)):
        if len(goals[n]) == 0:
            emptyGoals.append(n)
    
    return emptyPoses, emptyGoals

# Plotting the data
def plotData(param):

    # Initialize the figure
    fig, ax = plot.subplots(figsize=(10,10))

    # Define the origin
    origin_x = 0
    origin_y = 0

    if param == 'color_matching':
        for n in successIndices:
            # Colors of goals and treks should match -- if not, something is wrong
            colors = list(mcolors.CSS4_COLORS)
            num_colors = len(colors)
            
            # Separate the x and y positions into lists and plot the path
            time, pos, quat, euler = zip(*poses[n])
            pos = np.asarray(pos, dtype=float)
            poseX = pos[:,0]
            poseY = pos[:,1]
            poseXn = [((-1*y) - origin_y) for y in poseY]
            poseYn = [(t+origin_x) for t in poseX]
            ax.plot(poseXn, poseYn, str(colors[n%num_colors]))

            # Find the x and y position of the goal and plot it
            goalX, goalY = goals[n]
            goalXn = (goalY * -1) - origin_y
            goalYn = goalX + origin_x
            ax.scatter(goalXn, goalYn, c=colors[n%num_colors], zorder=1)
    
    if param == 'successes':
        for n in successIndices:
            # Separate the x and y positions into lists and plot the path
            time, pos, quat, euler = zip(*poses[n])
            pos = np.asarray(pos, dtype=float)
            poseX = pos[:,0]
            poseY = pos[:,1]
            poseXn = [((-1*y) - origin_y) for y in poseY]
            poseYn = [(t+origin_x) for t in poseX]
            ax.plot(poseXn, poseYn, 'mediumseagreen')

            # Find the x and y position of the goal and plot it
            goalX, goalY = goals[n]
            goalXn = (goalY * -1) - origin_y
            goalYn = goalX + origin_x
            ax.scatter(goalXn, goalYn, c='g', zorder=1)
    
    if param == 'failures':
        colors = list(mcolors.TABLEAU_COLORS)
        num_colors = len(colors)
        counter = 0
        for n in failureIndices:
            # Separate the x and y positions into lists and plot the path
            time, pos, quat, euler = zip(*poses[n])
            pos = np.asarray(pos, dtype=float)
            poseX = pos[:,0]
            poseY = pos[:,1]
            poseXn = [((-1*y) - origin_y) for y in poseY]
            poseYn = [(t+origin_x) for t in poseX]
            ax.plot(poseXn, poseYn, colors[counter%num_colors])

            # Find the x and y position of the goal and plot it
            goalX, goalY = goals[n]
            goalXn = (goalY * -1) - origin_y
            goalYn = goalX + origin_x
            ax.scatter(goalXn, goalYn, c=colors[counter%num_colors], marker='x', zorder=1)

            counter += 1
            
        ax.legend(failureIndices, title='Trial Number')

    # Specify the axes of the plot:
    size = 129/2
    xmin, ymin, xmax, ymax = -size, -size, size, size
    ax.set(xlim=(xmin-1, xmax+1), ylim=(ymin-1, ymax+1), aspect='equal')
    ticks_freq = 1

    # Set bottom and left spines as x and y axes of coordinate system
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_position('zero')

    # Remove top and right spines:
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Draw grid lines:
    ax.grid(which='major', color='grey', linewidth=1, linestyle='-', alpha=0.2)
    ax.grid(which='minor', color='grey', linewidth=1, linestyle='-', alpha=0.2)

    # Import image of terrain for added insight:
    img = plot.imread(Path.cwd().joinpath('terrain_map.jpeg'))
    ax.imshow(img, zorder=0, extent=[xmin, xmax, ymin, ymax])

    plot.show()

def failureEval():
    for index in failureIndices:
        print('\nTRIAL ' + str(index))

        poseData = poses[index]
        goal = goals[index]

        # Store data in appropriate arrays
        time, pos, quat, euler = zip(*poseData)
        time = np.asarray(time, dtype=int)
        pos = np.asarray(pos, dtype=float)
        quat = np.asarray(quat, dtype=float)
        euler = np.asarray(euler, dtype=float)

        # Find if the rover's roll and pitch exceed a certain value
        euler_deg = euler * 180/np.pi
        tipped = False
        for i in range(len(euler_deg)):
            if np.abs(euler_deg[i][0]) > 10 or np.abs(euler_deg[i][1]) > 10:
                tipped = True
        
        print('Tipped? ' + str(tipped))

        # Determine if the rover is heading towards the goal
        start_dist = np.linalg.norm(pos[0,0:2] - goal)
        final_dist = np.linalg.norm(pos[-1,0:2] - goal)
        if final_dist < start_dist:
            print('Closer to goal at completion of trial')
        else:
            print('Farther from goal at completion of trial')

        # Find where the rover is pausing for a long time
        pauses = []
        pause_count = 0
        pause_start = 0
        moving_at_end = True
        for i in range(len(pos)-1):
            if np.isclose(pos[i], pos[i+1]).all() and pause_count == 0:
                pause_start = time[i]
                pause_count += 1
            if np.isclose(pos[i], pos[i+1]).all():
                pause_count += 1
            else:
                if pause_count > 1000:
                    pauses.append([time[i]-pause_start, pause_start, pos[i]])
                pause_count = 0

        if pause_count > 1000:
            pauses.append([time[i]-pause_start, pause_start, pos[i]])
            moving_at_end = False

        for i in range(len(pauses)):
            duration = str(pauses[i][0]/1e9)
            start_time = str(pauses[i][1]/1e9)
            end_time = str(pauses[i][1]/1e9 + pauses[i][0]/1e9)
            location = str(pauses[i][2])
            print('Paused at ' + location + ' for ' + duration +
                    ' seconds, starting at ' + start_time +
                    ' seconds and ending at ' + end_time + ' seconds')

        print('Still moving at completion of the trial? ' + str(moving_at_end))

def main():

    readPoseFileVerbose()
    if DEBUG:
        print('Number of poses: {}'.format(len(poses)))

    readGoalFile()
    if DEBUG:
        print('Number of goals: {}'.format(len(goals)))

    emptyPoses, emptyGoals = findEmptyTrials()
    print('Trials without path data: ' + ', '.join('{}'.format(s) for s in emptyPoses))
    print('Trials without goal data: ' + ', '.join('{}'.format(s) for s in emptyGoals))

    print('\nTotal Successes: {}'.format(len(successIndices)))
    print('Success Indices:\n' + ', '.join('{}'.format(s) for s in successIndices))
    print('\nTotal Failures: {}'.format(len(failureIndices)))
    print('Failure Indices:\n' + ', '.join('{}'.format(s) for s in failureIndices))

    failureEval()

    plotData('successes')
    plotData('failures')

if __name__ == "__main__":
    main()