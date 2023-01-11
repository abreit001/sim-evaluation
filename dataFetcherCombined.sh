#!/bin/bash

source ~/Documents/autonomy/devel/setup.bash

roscore&
sleep 3

# Set the topics for data retrieval:
goalTime='/rover_executive/goal_command/timestamp'
goalX='/rover_executive/goal_command/origin_x_meters'
goalY='/rover_executive/goal_command/origin_y_meters'
goal='/rover_executive/goal_command'
reachedX='/rover_executive/goal_reached/origin_x_meters'
reachedY='/rover_executive/goal_reached/origin_y_meters'
reached='/rover_executive/goal_reached'
poseTopic='/odometry/filtered/pose/pose'
drive_arcs='/navigator/drive_arc'
drive_arc_costs='/navigator/drive_arc_costs'

goalFile=${1}
poseFile=${2}
# arcFile=${3}

# Clear files
echo
read -p "Clear ${1}, ${2}, and ${3}? y/n " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
    >${goalFile}
    >${poseFile}
    # >${arcFile}
fi

counter=0
path=/media/abreitfe/MOSDART/sim_tests

# Grab goal data from the "data" rosbag:
while [ $counter -lt 50 ]
do
  # Play the rosbag
  rosbag play -r 1000 ${path}/trial_${counter}_*

  # Record which trial this is in all files
  echo "Trial ${counter}" >>${goalFile}
  echo "Trial ${counter}" >>${poseFile}
  # echo "Trial ${counter}" >>${arcFile}

  # Record the path
  rostopic echo -p -b ${path}/trial_${counter}_* $poseTopic>>${poseFile}

  # Record the goal
  echo 'Goals Sent' >>${goalFile}
  rostopic echo -p -b ${path}/trial_${counter}_* $goal >>${goalFile}
  # rostopic echo -p -b ${path}/trial_${counter}_* $goalY >>${goalFile}
  
  # Record the goal reached topic (this records nothing if the goal was unreached)
  echo 'Goals Reached' >>${goalFile}
  rostopic echo -p -b ${path}/trial_${counter}_* $reached >>${goalFile}
  # rostopic echo -p -b ${path}/trial_${counter}_* $reachedY >>${goalFile}

  # Record the arcs and arc costs
  # rostopic echo -p -b ${path}/trial_${counter}_* $drive_arcs >>${arcFile}
  # rostopic echo -p -b ${path}/trial_${counter}_* $drive_arc_costs >>${arcFile}
  
  # Advance the counter
  ((counter++))
done

killall -9 rosmaster
