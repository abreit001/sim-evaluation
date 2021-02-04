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
# Initialize dictionaries that will hold the goals, poses, and arc info for each trial
goals_sent = {}
goals_reached = {}
poses = {}
arcs_curr = {}
arcs_eval = {}

# Organize pose data by appending all pose data coordinates into a path
# represented as a list. Upon completing the path, add it into a dictionary
# of poses (a dictionary of lists)
def readPoseFile():
    if poseFile == '':
        print('No pose file given')
        return

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
    if goalFile == '':
        print('No goal file given')
        return

    with open(os.getcwd()+'/'+goalFile) as goal_csv:
        
        print('Opened goal file')
        fileReader = csv.reader(goal_csv)
        
        currentList = []
        trial_num = 0
        recording_goals_sent = False

        for line in fileReader:
            # Assume we start at trial number 0
            if line[0] == 'Trial 0':
                continue
            
            # Skip over this line
            elif line[0] == '%time':
                continue

            elif line[0] == 'Goals Sent':
                recording_goals_sent = True

            # Add the goals sent to the goals_sent dictionary
            elif line[0] == 'Goals Reached':
                goals_sent[trial_num] = currentList
                currentList = []
                recording_goals_sent = False
            
            # When we get to the start of another trial, add the current list
            # to the goals_reached dictionary and update the trial number
            elif line[0][0:5] == 'Trial':
                goals_reached[trial_num] = currentList
                currentList = []
                trial_num = int(line[0][5:])
            
            # Add goal information to the current list,
            # fixing the formatting for the goal reached topic
            else:
                if recording_goals_sent:
                    currentList.append(np.array([float(line[1]), float(line[2])]))
                else:
                    currentList.append(np.array([float(line[2]), -float(line[1])]))
        
        # Add the last list to the correct dictionary
        if recording_goals_sent == True:
            goals_sent[trial_num] = currentList
        else:
            goals_reached[trial_num] = currentList

    # Distribute the data according to whether or not each goal has a
    # corresponding 'goalReached' tuple. If it does, it is a success.
    for num, currentSentList in goals_sent.items():
        currentReachedList = goals_reached[num]
        if len(currentSentList) == 0:
            continue
        
        failureFlag = False
        for i in range(len(currentSentList)):
            tempSent = currentSentList[i]
            # If there is no corresponding reached goal, this is a failure
            try:
                tempReached = currentReachedList[i]
            except:
                failureFlag = True
                break
            
            # Check if the goal reached and the actual goal are close enough
            if not (np.abs(tempSent[0] - tempReached[0]) < 0.001 and np.abs(tempSent[1] - tempReached[1]) < 0.001):
                failureFlag = True
        
        if failureFlag:
            failureIndices.append(num)
        else:
            successIndices.append(num)

# Get the arc data for each trial
def readArcFile():
    if arcFile == '':
        print('No arc file given')
        return

    with open(os.getcwd()+'/'+arcFile) as arc_csv:
        
        print('Opened arc file')
        fileReader = csv.reader(arc_csv)
        
        currentList = []
        trial_num = 0

        recording_current = False

        for line in fileReader:
            # Assume we start at trial number 0
            if line[0] == 'Trial 0':
                continue
            
            # if line[0] == 'Trial 4':
            #     break

            # Skip over this line if we have not added the current arcs to the list
            elif line[0] == '%time' and recording_current == False:
                recording_current = True

            # Add the current arcs to the arcs_curr dictionary
            elif line[0] == '%time' and recording_current == True:
                arcs_curr[trial_num] = currentList
                currentList = []
                recording_current = False
            
            # When we get to the start of another trial, add the current list
            # to the arcs_eval dictionary and update the trial number
            elif line[0][0:5] == 'Trial':
                arcs_eval[trial_num] = currentList
                currentList = []
                trial_num = int(line[0][5:])
            
            # Add arc information to the current list
            else:
                float_line = [float(i) for i in line]
                currentList.append(float_line)
        
        # Add the last list to the correct dictionary
        if recording_current == True:
            arcs_curr[trial_num] = currentList
        else:
            arcs_eval[trial_num] = currentList

def findEmptyTrials():
    emptyPoses = []
    emptyGoalsSent = []
    emptyGoalsReached = []
    emptyArcsCurr = []
    emptyArcsEval = []

    # Find any empty path data
    for n in range(len(poses)):
        if len(poses[n]) == 0:
            emptyPoses.append(n)

    # Find any empty goal data
    for n in range(len(goals_sent)):
        if len(goals_sent[n]) == 0:
            emptyGoalsSent.append(n)
    for n in range(len(goals_reached)):
        if len(goals_reached[n]) == 0:
            emptyGoalsReached.append(n)
    
    # Find any empty arc data
    for n in range(len(arcs_curr)):
        if len(arcs_curr[n]) == 0:
            emptyArcsCurr.append(n)
    for n in range(len(arcs_eval)):
        if len(arcs_eval[n]) == 0:
            emptyArcsEval.append(n)
    
    return emptyPoses, emptyGoalsSent, emptyGoalsReached, emptyArcsCurr, emptyArcsEval

def visualizeCurrArcs(trial_num):
    currentList = arcs_curr[trial_num]
    currentArray = np.array(currentList)
    # Convert time to seconds
    time = currentArray.T[0]
    time = (time - time[0])/1e9
    # Remap radius of 1000 to radius of 10 for visualization purposes
    for i in range(len(currentArray)):
        if currentArray[i][1] == 1000:
            currentArray[i][1] = 10
    radii = currentArray.T[1]
    plot.scatter(time, radii, s=3)
    plot.title('Arcs Taken')
    plot.xlabel('time (s)')
    plot.ylabel('radius (m)')
    plot.show()

def visualizeArcOptions(trial_num, param, plot_separate):
    currentList = arcs_eval[trial_num]
    currentArray = np.array(currentList)
    time = currentArray.T[0]
    time = (time - time[0])/1e9
    
    if param == 'near':
        title = 'Near Field Arc Costs'
        skip = 3
    elif param == 'far':
        title = 'Far Field Arc Costs'
        skip = 12
    elif param == 'total':
        title = 'Total Arc Costs'
        skip = 21

    plot.figure(figsize=(10,10))
    for i in range(9):
        cost = currentArray.T[i+skip]
        if plot_separate:
            plot.subplot(3, 3, i+1)
            plot.scatter(time, cost, s=1)
        else:
            plot.scatter(time, cost)
            plot.title('Arc ' + str(i))
        plot.xlabel('time (s)')
        plot.ylabel('cost')
        plot.ylim([0, 1000])
    
    if plot_separate:
        plot.suptitle(title)
    else:
        plot.legend(['Arc 0', 'Arc 1', 'Arc 2', 'Arc 3', 'Arc 4', 'Arc 5', 'Arc 6', 'Arc 7', 'Arc 8'], title='Arcs')
    plot.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot.show()

# Plotting the data
def plotData(param):

    # Initialize the figure
    fig, ax = plot.subplots(figsize=(10,10))

    # Define the origin
    origin_x = 0
    origin_y = 0

    if param == 'color_matching':
        for n in range(len(poses)):
            # Colors of goals and treks should match -- if not, something is wrong
            colors = list(mcolors.CSS4_COLORS)
            num_colors = len(colors)
            
            # Separate the x and y positions into lists and plot the path
            if len(poses[n]) > 0:
                time, pos, quat, euler = zip(*poses[n])
                pos = np.asarray(pos, dtype=float)
                poseX = pos[:,0]
                poseY = pos[:,1]
                poseXn = [((-1*y) - origin_y) for y in poseY]
                poseYn = [(t+origin_x) for t in poseX]
                ax.plot(poseXn, poseYn, str(colors[n%num_colors]))

            # Find the x and y position of the goal and plot it
            if len(goals_sent[n]) > 0:
                goalX = np.array(goals_sent[n])[:,0]
                goalY = np.array(goals_sent[n])[:,1]
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
            goalX = np.array(goals_sent[n])[:,0]
            goalY = np.array(goals_sent[n])[:,1]
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
            goalX = np.array(goals_sent[n])[:,0]
            goalY = np.array(goals_sent[n])[:,1]
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
        goalSentData = goals_sent[index]
        goalReachedData = goals_reached[index]

        # Store data in appropriate arrays
        time, pos, quat, euler = zip(*poseData)
        time = np.asarray(time, dtype=int)
        pos = np.asarray(pos, dtype=float)
        quat = np.asarray(quat, dtype=float)
        euler = np.asarray(euler, dtype=float)

        # Find if the rover's roll and pitch exceed a certain value
        euler_deg = euler * 180/np.pi
        tipped = False
        tips = []
        tipped_time = []
        for i in range(len(euler_deg)):
            if np.abs(euler_deg[i][0]) > 10 or np.abs(euler_deg[i][1]) > 10:
                tipped = True
                tipped_time.append(time[i] - time[0])
            elif len(tipped_time) > 0:
                tips.append([tipped_time[0], tipped_time[-1]])
                tipped_time = []
        
        for i in range(len(tips)):
            print('Tipped at ' + str(tips[i][0]/1e9) + ' seconds to ' + str(tips[i][1]/1e9) + ' seconds')

        # Determine how many goals were reached
        reachedIndices = []
        for i in range(len(goalSentData)):
            tempSent = goalSentData[i]
            # If there is no corresponding reached goal, this is a failure
            try:
                tempReached = goalReachedData[i]
            except:
                continue
            
            # Check if the goal reached and the actual goal are close enough
            if np.abs(tempSent[0] - tempReached[0]) < 0.001 and np.abs(tempSent[1] - tempReached[1]) < 0.001:
                reachedIndices.append(i+1)
        
        print('Goals reached: ' + ', '.join('{}'.format(s) for s in reachedIndices) + ' (' + str(len(reachedIndices)) + '/' + str(len(goalSentData)) + ')')

        # Determine if the rover is heading towards the goal
        # start_dist = np.linalg.norm(pos[0,0:2] - goal)
        # final_dist = np.linalg.norm(pos[-1,0:2] - goal)
        # if final_dist < start_dist:
        #     print('Closer to goal at completion of trial')
        # else:
        #     print('Farther from goal at completion of trial')

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

        print('Total length of trial: ' + str((time[-1]-time[0])/1e9) + ' seconds')
        print('Still moving at completion of the trial? ' + str(moving_at_end))

def main():

    global goalFile
    global poseFile
    global arcFile

    try:
        goalFile = sys.argv[1]
    except:
        goalFile = ''
    try:
        poseFile = sys.argv[2]
    except:
        poseFile = ''
    try:
        arcFile = sys.argv[3]
    except:
        arcFile = ''

    readPoseFile()
    if DEBUG:
        print('Number of trials in pose file:           {}'.format(len(poses)))

    readGoalFile()
    if DEBUG:
        print('Number of trials in goals sent file:     {}'.format(len(goals_sent)))
        print('Number of trials in goals reached file:  {}'.format(len(goals_reached)))

    readArcFile()
    if DEBUG:
        print('Number of trials in arcs taken file:     {}'.format(len(arcs_curr)))
        print('Number of trials in arcs evaluated file: {}'.format(len(arcs_eval)))

    emptyPoses, emptyGoalsSent, emptyGoalsReached, emptyArcsCurr, emptyArcsEval = findEmptyTrials()
    print('')
    print('Trials without path data:           ' + ', '.join('{}'.format(s) for s in emptyPoses))
    print('Trials without goals sent data:     ' + ', '.join('{}'.format(s) for s in emptyGoalsSent))
    print('Trials without goals reached data:  ' + ', '.join('{}'.format(s) for s in emptyGoalsReached))
    print('Trials without current arc data:    ' + ', '.join('{}'.format(s) for s in emptyArcsCurr))
    print('Trials without arc evaluation data: ' + ', '.join('{}'.format(s) for s in emptyArcsEval))

    print('\nTotal Successes: {}'.format(len(successIndices)))
    print('Success Indices:\n' + ', '.join('{}'.format(s) for s in successIndices))
    print('\nTotal Failures: {}'.format(len(failureIndices)))
    print('Failure Indices:\n' + ', '.join('{}'.format(s) for s in failureIndices))

    # visualizeCurrArcs(1)
    # visualizeArcOptions(1, 'near', False)
    # visualizeArcOptions(1, 'far', False)
    # visualizeArcOptions(1, 'total', False)
    failureEval()

    # total = 0
    # for n in successIndices:
    #     if len(poses[n]) > 0:
    #         time, pos, quat, euler = zip(*poses[n])
    #         time = np.asarray(time, dtype=int)
    #         total += (time[-1]-time[0])/1e9
    # print(total/len(successIndices))

    plotData('successes')
    plotData('failures')

if __name__ == "__main__":
    main()
