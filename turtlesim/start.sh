#!/bin/bash

main_dir=$PWD

roslaunch turtlesim siu.launch &
ros_pid=$!

sleep 10

cd scripts
python turtle.py
cd $main_dir

kill $ros_pid
