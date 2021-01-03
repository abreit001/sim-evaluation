#!/bin/bash

source ~/Documents/autonomy/devel/setup.bash

roscore&


# 1.) Set the topics for data retrieval:
goalTime='/rover_executive/goal_command/timestamp'
goalX='/rover_executive/goal_command/origin_x_meters'
goalY='/rover_executive/goal_command/origin_y_meters'
reachedX='/rover_executive/goal_reached/origin_x_meters'
reachedY='/rover_executive/goal_reached/origin_y_meters'
poseTopic='/odometry/filtered/pose/pose'

goalFile=goalData
poseFile=poseData

counter=0
path=/media/usb/sim_trials
#: '
#2.) Grab goal data from the "data" rosbag:
#'
# while [ $counter -lt 100 ]
# do
#   #NOTE: The paths in the proceeding ros* calls need to be specified to the user's system.
#   rosbag play -r 1000 ${path}/trial_${counter}_*
#   if [[ $counter -eq 0 ]]; then
#     rostopic echo -p -b ${path}/trial_${counter}_* $goalX >${goalFile}.csv
#     rostopic echo -p -b ${path}/trial_${counter}_* $goalY >>${goalFile}.csv
#   else
#     rostopic echo -p -b ${path}/trial_${counter}_* $goalX >>${goalFile}.csv
#     rostopic echo -p -b ${path}/trial_${counter}_* $goalY >>${goalFile}.csv
#   fi

# #  ' : NOTE: The following if-else statement is meant to record goals that are reached but skip goals that are not (as
# #   indicated by empty rostopics). But it turns out that these 'empty' rostopics still contain data, and the
# #   '-n' and '-z' flags do not work. That said, although the 'if' portion of this control is always executed,
# #   if the topic is empty, only the '111111' string is logged. The missing data can be accounted for by counting lines.
# #  '
#   if [[ -n "rostopic echo ${path}/trial_${counter}_* $reachedX" ]]; then
#     rostopic echo -p -b ${path}/trial_${counter}_* $reachedX >>${goalFile}.csv
#     rostopic echo -p -b ${path}/trial_${counter}_* $reachedY >>${goalFile}.csv
#     echo 111111 >>${goalFile}.csv
#   else
#     echo 000000 >>${goalFile}.csv
#   fi
#   ((counter++))
# done

# ' : 3.) Get the pose data. This process is separate from collecting goal data
#         for the sake of portability--there are far more pose messages than
#         goal messages, and we may want to separately collect them in the future.
#         Output should be found in the .csv designated at the beginning of this script.
# '
while [ $counter -lt 100 ]
do
  #NOTE: The paths in the proceeding ros* calls need to be specified to the user's system.
  rosbag play -r 1000 ${path}/trial_${counter}_*

  if [[ counter -eq 0 ]]; then
    rostopic echo -p -b ${path}/trial_${counter}_* $poseTopic>${poseFile}.csv
  else
    echo '111111111'>>${poseFile}.csv
    rostopic echo -p -b ${path}/trial_${counter}_* $poseTopic>>${poseFile}.csv
  fi
  ((counter++))
done

killall -9 rosmaster
