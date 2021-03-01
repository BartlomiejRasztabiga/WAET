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
    if not turtle_api.hasTurtle('turtle2'):
        turtle_api.spawnTurtle('turtle2',turtlesim.msg.Pose(x=10,y=5,theta=0))
    k = 0
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
        cmd.linear.x = 0.0 
        cmd.linear.y = 0.0
        cmd.angular.z = 0.06 # theta
        if turtle_api.hasTurtle('turtle2'):
            print "CMD status: ", turtle_api.setVel('turtle2', cmd)
    	print '---------------------------------'
        img_response = turtle_api.readCamera(name='turtle2', frame_pixel_size = 200, cell_count=16, goal = Pose(x=10,y=5,theta=0))
        i = 0
        print "Matrix: " 
        for row in img_response.m_rows:
            j = 0
            for cell in row.cells:
                print "\tCELL_"+str(i)+"_"+str(j)+":" 
                print "\t\tR: ", cell.red
                print "\t\tG: ", cell.green
                print "\t\tB: ", cell.blue
                print "\t\tDist: ", cell.distance
                j +=1
            i += 1
        cv_image = bridge.imgmsg_to_cv2(img_response.image, desired_encoding='passthrough')
        cv2.imwrite('/tmp/cv_out.jpg', cv_image) 
    	rate.sleep()
        k += 1 
