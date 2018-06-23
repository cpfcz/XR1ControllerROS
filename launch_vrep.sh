#!/bin/bash  
echo "Launching Stuff"  
roscore &
echo "Core started" 
cd V-REP
./vrep.sh scenes/Ginger.ttt  
wait 
cd ..
ps -A |grep roscore |grep -v grep|cut -c 9-15|xargs kill -1  