#!/usr/bin/env python
# encoding: utf8
import rospy
import turtlesim
from turtlesim.msg import Pose
from turtlesim.srv  import SetPenRequest
from TurtlesimSIU import TurtlesimSIU 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import math
from cv_bridge import CvBridge
import cv2
import signal
import sys
import numpy as np
import pylab as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

VISUALIZE = True

def signal_handler(sig, frame):
    print ("Terminating")
    sys.exit(0)

if __name__ == "__main__":
    # Initialize ROS node
    signal.signal(signal.SIGINT, signal_handler)
    bridge = CvBridge()
    rospy.init_node('siu_example', anonymous=False)
    turtle_api = TurtlesimSIU.TurtlesimSIU()
    rate = rospy.Rate(1)
    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)
    #set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)
    if turtle_api.hasTurtle('turtle1'):
        turtle_api.killTurtle('turtle1')
        # rospy.sleep(2)
    if not turtle_api.hasTurtle('turtle1'):
        turtle_api.spawnTurtle('turtle1',turtlesim.msg.Pose(x=10,y=5,theta=0))
    color_api = TurtlesimSIU.ColorSensor('turtle1')
    print turtle_api.pixelsToScale()
    #
    # translate and rotate afterwards
    #
    turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=20,y=1,theta=1), mode='absolute')
    #
    # rotate and translate afterwards
    #
    turtle_api.setPose(turtle_name='turtle1', pose=turtlesim.msg.Pose(x=2,y=1,theta=1), mode='relative')

    while not rospy.is_shutdown():
        print ('POSE:')
        print ('\t {}',turtle_api.getPose('turtle1').x)
        print ('\t {}',turtle_api.getPose('turtle1').y)
        print ('\t {}',turtle_api.getPose('turtle1').theta)
        print( '\t {}',turtle_api.getPose('turtle1').linear_velocity)
        print( '\t {}',turtle_api.getPose('turtle1').angular_velocity)

        print( turtle_api.setPen('turtle1',set_pen_req))
        cmd = Twist()
        cmd.linear.x = 0.0 
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0 # theta
        if turtle_api.hasTurtle('turtle1'):
            print( "CMD status:  {}", turtle_api.setVel('turtle1', cmd))
        print( '---------------------------------')
        rate.sleep()