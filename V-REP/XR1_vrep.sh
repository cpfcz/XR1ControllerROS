#!/bin/bash  
echo "Launching Stuff"  
roscore &
echo "Core started" 
rqt --standalone vrep_test &
./vrep.sh scenes/XR1_Vanilla.ttt  
wait 
ps -A |grep roscore |grep -v grep|cut -c 9-15|xargs kill -1  
