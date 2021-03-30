#!/bin/bash

main_dir=$PWD

roslaunch turtlesim siu.launch &
ros_pid=$!

sleep 3

cd scripts
python turtle.py 1 1
cd $main_dir

kill $ros_pid
