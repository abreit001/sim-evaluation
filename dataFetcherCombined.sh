#!/bin/bash

source ~/Documents/autonomy/devel/setup.bash

roscore&

# Set the topics for data retrieval:
goalTime='/rover_executive/goal_command/timestamp'
goalX='/rover_executive/goal_command/origin_x_meters'
goalY='/rover_executive/goal_command/origin_y_meters'
reachedX='/rover_executive/goal_reached/origin_x_meters'
reachedY='/rover_executive/goal_reached/origin_y_meters'
poseTopic='/odometry/filtered/pose/pose'

goalFile=goalDataOutAndBack.csv
poseFile=poseDataOutAndBack.csv

# Clear files
>${goalFile}
>${poseFile}

counter=0
path=/media/usb/sim_out_and_back

# Grab goal data from the "data" rosbag:
while [ $counter -lt 100 ]
do
  # Play the rosbag
  rosbag play -r 1000 ${path}/trial_${counter}_*

  # Record which trial this is in both files
  echo "Trial ${counter}" >>${goalFile}
  echo "Trial ${counter}" >>${poseFile}

  # Record the path
  rostopic echo -p -b ${path}/trial_${counter}_* $poseTopic>>${poseFile}

  # Record the goal
  rostopic echo -p -b ${path}/trial_${counter}_* $goalX >>${goalFile}
  rostopic echo -p -b ${path}/trial_${counter}_* $goalY >>${goalFile}
  
  # Record the goal reached topic (this records nothing if the goal was unreached)
  rostopic echo -p -b ${path}/trial_${counter}_* $reachedX >>${goalFile}
  rostopic echo -p -b ${path}/trial_${counter}_* $reachedY >>${goalFile}
  
  # Advance the counter
  ((counter++))
done

killall -9 rosmaster
