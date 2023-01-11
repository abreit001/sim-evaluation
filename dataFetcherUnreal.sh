#!/bin/bash

roscore&
sleep 3

# Set the topics for data retrieval:
goalTopic='/goal_for_cfs_planner'
goalReachedTopic='/goal_reached_for_cfs_planner'
poseTopic='/odometry/filtered/pose/pose'

path=${1}
num_tests=${2}
goalFile="goal_data_${3}.csv"
poseFile="pose_data_${3}.csv"

# Clear files
echo
read -p "Clear ${goalFile} and ${poseFile}? y/n " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
    >${goalFile}
    >${poseFile}
fi

counter=0

# Grab goal data from the "data" rosbag:
while [ $counter -lt $num_tests ]
do
  # Play the rosbag
  # rosbag play -r 1000 ${path}/trial_${counter}_*.bag

  # Record which trial this is in all files
  echo "Trial ${counter}" >>${goalFile}
  echo "Trial ${counter}" >>${poseFile}

  # Copy pose data
  rostopic echo -p -b ${path}/trial_${counter}_*.bag $poseTopic >>${poseFile}

  # Record the goal
  echo 'Goals Sent' >>${goalFile}
  rostopic echo -p -b ${path}/trial_${counter}_*.bag $goalTopic >>${goalFile}
  
  # Record the goal reached topic (this records nothing if the goal was unreached)
  echo 'Goals Reached' >>${goalFile}
  rostopic echo -p -b ${path}/trial_${counter}_*.bag $goalReachedTopic >>${goalFile}

  # Increment test counter
  ((counter++))
done

killall -9 rosmaster
