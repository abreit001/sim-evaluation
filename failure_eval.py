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

files = []
directory = sys.argv[1]
for filename in os.listdir(directory):
    if 'poseData' in filename:
        files.append(filename)

for filename in files:
    print('\n')
    print('TRIAL ' + filename[15] + filename[16])
    data = []

    # Open the csv file and read all rows into data
    with open(directory + filename) as csvfile:
        file_reader = csv.reader(csvfile, delimiter=',')
        for row in file_reader:
            data.append(row)

    # Find the range of values we want to store
    start = 0
    start_found = False
    end = 0
    end_found = False
    for i in range(len(data)):
        if data[i][0] == '%time':
            start = i + 1
            start_found = True
        elif (data[i][0] == '%time' or data[i][0] == '111111111') and start_found:
            end = i - 1
            end_found = True
            break
    if not end_found:
        end = len(data)-1

    # Store data in appropriate arrays
    time = np.zeros(end-start+1)
    pos = np.zeros((end-start+1, 3))
    quat = np.zeros((end-start+1, 4))
    euler = np.zeros((end-start+1, 3))

    for i in range(start, end+1):
        time[i-start] = data[i][0]
        pos[i-start] = data[i][1:4]
        quat[i-start] = data[i][4:8]
        # print(quat[i-start])
        euler[i-start] = euler_from_quaternion(quat[i-start])
        quat_check = quaternion_from_euler(euler[i-start,0], euler[i-start,1], euler[i-start,2])
        # print(quat_check)
        # print(euler[i-start])

    # Find if the rover's roll and pitch exceed a certain value
    euler_deg = euler * 180/np.pi
    # print(np.max(euler[:,0]))
    # print(np.max(euler[:,1]))
    # print(np.max(euler[:,2]))
    # print(np.min(euler[:,0]))
    # print(np.min(euler[:,1]))
    # print(np.min(euler[:,2]))
    tipped = False
    for i in range(len(euler_deg)):
        if np.abs(euler_deg[i][0]) > 10 or np.abs(euler_deg[i][1]) > 10:
            tipped = True
    
    print('Tipped? ' + str(tipped))

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