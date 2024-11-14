#!/bin/bash

sim_vehicle.py -v ArduCopter --console --map --moddebug 3 &
PY_PID="$!"

# Launch the simulator, unless it is already running
if [ -z $(pgrep main) ]
then
  roslaunch dna sim_pc.launch &
  ROS_PID="$!"
  echo $ROS_PID
  sleep 1
else
  ROS_PID=""
fi

sleep 5
rosrun mavros mavsys rate --raw-controller 0
sleep 0.1
rosrun mavros mavsys rate --raw-sensors 0
sleep 0.1
rosrun mavros mavsys rate --rc-channels 0
sleep 0.1
rosrun mavros mavsys rate --extra2 0
sleep 0.1
rosrun mavros mavsys rate --extra3 0

while true
do
  rosrun mavros mavsys rate --position 100
  rosrun mavros mavsys rate --extra1 100
  sleep 3
done