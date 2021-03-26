#!/usr/bin/env python
# encoding: utf8
import rospy
import turtlesim
from turtlesim.msg import Pose
from turtlesim.srv import SetPenRequest
from TurtlesimSIU import TurtlesimSIU
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from pynput.keyboard import Listener, Key
import math
import signal
import sys
import numpy as np

VISUALIZE = True

base_x = 10
base_y = 10

width = 5
length = 5

current_msg = "neutral"
currently_pressed = set()


def on_press(key):
    global current_msg
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k not in currently_pressed:
        currently_pressed.add(k)
    current_msg = k


def on_release(key):
    global current_msg
    current_msg = "neutral"

    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in currently_pressed:
        currently_pressed.remove(k)

    # stop on PAUSE
    if key == Key.pause:
        print("quit on PAUSE")
        return False


def signal_handler(sig, frame):
    print("Terminating")
    sys.exit(0)


def spawn_turtles():
    if turtle_api.hasTurtle('turtle1'):
        turtle_api.killTurtle('turtle1')
    if not turtle_api.hasTurtle('turtle1'):
        turtle_api.spawnTurtle('turtle1', turtlesim.msg.Pose(x=base_x, y=base_y, theta=0))

    if turtle_api.hasTurtle('turtle2'):
        turtle_api.killTurtle('turtle2')
    if not turtle_api.hasTurtle('turtle2'):
        turtle_api.spawnTurtle('turtle2', turtlesim.msg.Pose(x=base_x + length // 2, y=base_y + width // 2, theta=0))

    if turtle_api.hasTurtle('turtle3'):
        turtle_api.killTurtle('turtle3')
    if not turtle_api.hasTurtle('turtle3'):
        turtle_api.spawnTurtle('turtle3', turtlesim.msg.Pose(x=base_x + length // 2, y=base_y - width // 2, theta=0))

    if turtle_api.hasTurtle('turtle4'):
        turtle_api.killTurtle('turtle4')
    if not turtle_api.hasTurtle('turtle4'):
        turtle_api.spawnTurtle('turtle4', turtlesim.msg.Pose(x=base_x - length // 2, y=base_y + width // 2, theta=0))

    if turtle_api.hasTurtle('turtle5'):
        turtle_api.killTurtle('turtle5')
    if not turtle_api.hasTurtle('turtle5'):
        turtle_api.spawnTurtle('turtle5', turtlesim.msg.Pose(x=base_x - length // 2, y=base_y - width // 2, theta=0))


def move_turtles():
    print(currently_pressed)
    if 'w' in currently_pressed:
        move_forward()
    if 's' in currently_pressed:
        move_backward()
    if 'a' in currently_pressed:
        turn_left()
    if 'd' in currently_pressed:
        turn_right()


def move_forward():
    turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle2', pose=turtlesim.msg.Pose(x=0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle3', pose=turtlesim.msg.Pose(x=0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle4', pose=turtlesim.msg.Pose(x=0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle5', pose=turtlesim.msg.Pose(x=0.1, y=0, theta=0), mode='relative')


def move_backward():
    turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=-0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle2', pose=turtlesim.msg.Pose(x=-0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle3', pose=turtlesim.msg.Pose(x=-0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle4', pose=turtlesim.msg.Pose(x=-0.1, y=0, theta=0), mode='relative')
    turtle_api.setPose(turtle_name='turtle5', pose=turtlesim.msg.Pose(x=-0.1, y=0, theta=0), mode='relative')


def turn_left():
    turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=-0, y=0, theta=0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle2', pose=turtlesim.msg.Pose(x=-0, y=0, theta=0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle3', pose=turtlesim.msg.Pose(x=-0, y=0, theta=0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle4', pose=turtlesim.msg.Pose(x=-0, y=0, theta=0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle5', pose=turtlesim.msg.Pose(x=-0, y=0, theta=0.1), mode='relative')


def turn_right():
    turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=-0, y=0, theta=-0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle2', pose=turtlesim.msg.Pose(x=-0, y=0, theta=-0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle3', pose=turtlesim.msg.Pose(x=-0, y=0, theta=-0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle4', pose=turtlesim.msg.Pose(x=-0, y=0, theta=-0.1), mode='relative')
    turtle_api.setPose(turtle_name='turtle5', pose=turtlesim.msg.Pose(x=-0, y=0, theta=-0.1), mode='relative')


if __name__ == "__main__":
    # Initialize ROS node
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('siu_example', anonymous=False)
    turtle_api = TurtlesimSIU.TurtlesimSIU()
    rate = rospy.Rate(1)
    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)
    # set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)

    spawn_turtles()

    listener = Listener(on_press=on_press, on_release=on_release, suppress=False)
    listener.start()

    print(turtle_api.pixelsToScale())
    #
    # translate and rotate afterwards
    #
    # turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=20,y=1,theta=1), mode='absolute')
    #
    # rotate and translate afterwards

    # turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=2,y=1,theta=1), mode='relative')

    while listener.running and not rospy.is_shutdown():
        # turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=0.1,y=0.1,theta=0.2), mode='relative')
        move_turtles()
        # print('POSE:')
        # print('\t {}', turtle_api.getPose('turtle1').x)
        # print('\t {}', turtle_api.getPose('turtle1').y)
        # print('\t {}', turtle_api.getPose('turtle1').theta)
        # print('\t {}', turtle_api.getPose('turtle1').linear_velocity)
        # print('\t {}', turtle_api.getPose('turtle1').angular_velocity)

        print(turtle_api.setPen('turtle1', set_pen_req))
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0  # theta
        if turtle_api.hasTurtle('turtle1'):
            print("CMD status:  {}", turtle_api.setVel('turtle1', cmd))
        print('---------------------------------')
        rate.sleep()
