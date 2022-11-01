#!/bin/bash

# Pass number of rollouts as argument
if [ $1 ]
then
  N="$1"
else
  N=10
fi

# Set Flightmare Path if it is not set
if [ -z $FLIGHTMARE_PATH ]
then
  export FLIGHTMARE_PATH=$PWD/flightmare
fi

# Launch the simulator, unless it is already running
if [ -z $(pgrep visionsim_node) ]
then
  roslaunch envsim visionenv_sim.launch render:=True &
  ROS_PID="$!"
  echo $ROS_PID
  sleep 1
else
  ROS_PID=""
fi

SUMMARY_FILE="evaluation.yaml"

# Perform N evaluation runs
for i in $(eval echo {1..$N})
  do
  # Publish simulator reset
  rostopic pub /kingfisher/dodgeros_pilot/off std_msgs/Empty "{}" --once
  rostopic pub /kingfisher/dodgeros_pilot/reset_sim std_msgs/Empty "{}" --once
  rostopic pub /kingfisher/dodgeros_pilot/enable std_msgs/Bool "data: true" --once
  cd ./envtest/ros/
  
  case $((i%3)) in 
    0)
      rostopic pub /sampling_mode std_msgs/Int8 "data: 2" --once
      python3 benchmarking_node.py --policy=depth_based &
      PY_PID="$!"
      python3 run_competition.py --steering=True &
      COMP_PID="$!" 
      ;;
    1) 
      rostopic pub /sampling_mode std_msgs/Int8 "data: 0" --once
      python3 benchmarking_node.py &
      PY_PID="$!"
      python3 run_competition.py &
      COMP_PID="$!"
      ;;
  esac
  cd -

  sleep 0.5
  rostopic pub /kingfisher/start_navigation std_msgs/Empty "{}" --once

  # Wait until the evaluation script has finished
  while ps -p $PY_PID > /dev/null
  do
    sleep 1
  done
  cat "$SUMMARY_FILE" "./envtest/ros/summary.yaml" > "tmp.yaml"
  mv "tmp.yaml" "$SUMMARY_FILE"

  kill -SIGINT "$COMP_PID"
done

if [ $ROS_PID ]
then
  kill -SIGINT "$ROS_PID"
fi
