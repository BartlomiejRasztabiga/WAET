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

if __name__ == "__main__":
    # Initialize ROS node
    bridge = CvBridge()
    rospy.init_node('siu_example', anonymous=False)
    turtle_api = TurtlesimSIU.TurtlesimSIU()
    color_api = TurtlesimSIU.ColorSensor('turtle1')
    rate = rospy.Rate(1)
    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)
    #set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)
    turtle_api.spawnTurtle('turtle2',turtlesim.msg.Pose(x=19,y=12,theta=0))
    i = 0
    while not rospy.is_shutdown():
    	print 'POSE:'
    	print '\t',turtle_api.getPose('turtle1').x
    	print '\t',turtle_api.getPose('turtle1').y
    	print '\t',turtle_api.getPose('turtle1').theta
    	print '\t',turtle_api.getPose('turtle1').linear_velocity
    	print '\t',turtle_api.getPose('turtle1').angular_velocity
    	print 'SONAR:'
        #  turtle_api.readSonar( direction, FOV, min_range, max_range, turtle_name)                      
    	print '\t',turtle_api.readSonar(0, math.pi/2, 0.5, 2,'turtle1')
    	print 'COLOR:'
    	print '\t',color_api.check().r
    	print '\t',color_api.check().g
    	print '\t',color_api.check().b
    	print turtle_api.setPen('turtle1',set_pen_req)
        print turtle_api.getColisions(['turtle1', 'turtle2'], 1)
        cmd = Twist()
        cmd.linear.x = 0.0 + i*0.1
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0 # theta
        print "CMD status: ", turtle_api.setVel('turtle2', cmd)
    	print '---------------------------------'
        img_response = turtle_api.readCamera('turtle1')
        cv_image = bridge.imgmsg_to_cv2(img_response, desired_encoding='passthrough')
        cv2.imwrite('/tmp/cv_out.jpg', cv_image) 
    	rate.sleep()
        i += 1 
