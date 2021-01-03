import numpy as np
import scipy
import csv
import rospy
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import os
import sys

DEBUG = True

goalFile = 'goalData.csv'
poseFile = 'poseData.csv'

# Organize pose data by appending tuples of (time, position, orientation)
# represented as a list. Upon completing the trek, append it into a list
# of treks (ie. a list of lists):
poseTuple = []
with open(os.getcwd()+'/'+poseFile) as csv1:
    # print('opened file\n')
    fileReader = csv.reader(csv1)
    # 2b.) Collect all the relevant data:
    currentList = []
    for line in fileReader:
        if line[0] == '%time':
            continue
        elif line[0] == '111111111':
            poseTuple.append(currentList)
            currentList = []
        else:
            time = line[0]
            pos = line[1:4]
            quat = line[4:8]
            euler = euler_from_quaternion(quat)
            currentList.append((time, pos, quat, euler))
    poseTuple.append(currentList)

if DEBUG:
    print('\nposeTuple size: {}'.format(len(poseTuple)))

emptyTrials = []
# Find any empty pose data
for n in range(len(poseTuple)):
    if len(poseTuple[n]) == 0:
        emptyTrials.append(n)

goalData = []
successIndices = []
failureIndices = []
failures    = []
successes   = []
allTrials   = []
# Get the goal data:
with open(os.getcwd()+'/'+goalFile) as csv1:
    # print('opened file\n')
    fileReader = csv.reader(csv1)
    
    # Collect all the relevant data:
    for line in fileReader:
        if line[0] == '%time' or line[0] == '111111':
            continue
        else:
            goalData.append(float(line[1]))
    if DEBUG:
        print('goalData size: {}'.format(len(goalData)))
    
    # Distribute the data according to whether or not the tuple at data[n]
    # has a corresponing 'goalReached' tuple. If it does, put the coordinate
    # in the 'successes' list:
    n=0
    loopCount = 0
    while n < len(goalData)-1:
        allTrials.append((goalData[n], goalData[n+1]))
        # Skip over this index if we did not find data for this trial
        if loopCount in emptyTrials:
            loopCount = loopCount + 1
        # If the goal is close enough to the final point of the rover,
        # consider the trial a success (previously elif goalData[n] == goalData[n + 3] and goalData[n+1] == -1*goalData[n+2]:)
        else:
            try:
                if np.abs(goalData[n] - goalData[n + 3]) < 0.001 and np.abs(goalData[n+1] + goalData[n+2]) < 0.001:
                    successes.append((goalData[n], goalData[n+1]))
                    successIndices.append(loopCount)
                    n  = n + 4
                    loopCount = loopCount + 1
                else:
                    # If there is no corresponding 'goalReached' tuple, put the
                    # data point in the 'failures' list:
                    failures.append((goalData[n], goalData[n+1]))
                    failureIndices.append(loopCount)
                    n  = n + 2
                    loopCount = loopCount + 1
            except:
                # If there is no corresponding 'goalReached' tuple, put the
                # data point in the 'failures' list:
                failures.append((goalData[n], goalData[n+1]))
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

for index in failureIndices:
    print('\n')
    print('TRIAL ' + str(index))

    poseData = poseTuple[index]
    goal = np.asarray(allTrials[index])

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