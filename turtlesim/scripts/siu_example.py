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
    color_api = TurtlesimSIU.ColorSensor('turtle1')
    rate = rospy.Rate(1)
    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)
    #set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)
    if turtle_api.hasTurtle('turtle2'):
        turtle_api.killTurtle('turtle2')
        # rospy.sleep(2)
    if not turtle_api.hasTurtle('turtle2'):
        turtle_api.spawnTurtle('turtle2',turtlesim.msg.Pose(x=10,y=5,theta=0))
    k = 0
    if VISUALIZE:
        fig = plt.figure(figsize=(16,12))
        
        axR = fig.add_subplot(131)#plt.subplots(1,3)
        fig.suptitle("RGB matrices")
        axR.set_title("R")
        im_r = axR.imshow(np.zeros((4,4)), norm=plt.Normalize(vmin=-1, vmax=1),cmap='hot')
        # X, Y = np.meshgrid(np.arange(0,4,1),np.arange(0,4,1))
        # plt.pcolor(X, Y,np.zeros((4,4)),cmap='hot', vmin=-1, vmax=1)
        divider = make_axes_locatable(axR)
        cax = divider.append_axes('right', size='5%', pad=0.1)
        fig.colorbar(im_r,cax=cax, orientation='vertical')
        axG = fig.add_subplot(132)#plt.subplots(1,3)
        axG.set_title("G")
        im_g = axG.imshow(np.zeros((4,4)), norm=plt.Normalize(vmin=0, vmax=255),cmap='hot')
        divider = make_axes_locatable(axG)
        cax = divider.append_axes('right', size='5%', pad=0.1)
        fig.colorbar(im_g,cax=cax, orientation='vertical')
        axB = fig.add_subplot(133)#plt.subplots(1,3)
        axB.set_title("B")
        divider = make_axes_locatable(axB)
        cax = divider.append_axes('right', size='5%', pad=0.1)
        im_b = axB.imshow(np.zeros((4,4)), norm=plt.Normalize(vmin=-1, vmax=1),cmap='hot')
        fig.colorbar(im_b,cax=cax, orientation='vertical')
        fig.tight_layout(pad=1)
    while not rospy.is_shutdown():
        print ('POSE:')
        print ('\t {}',turtle_api.getPose('turtle1').x)
        print ('\t {}',turtle_api.getPose('turtle1').y)
        print ('\t {}',turtle_api.getPose('turtle1').theta)
        print( '\t {}',turtle_api.getPose('turtle1').linear_velocity)
        print( '\t {}',turtle_api.getPose('turtle1').angular_velocity)
        print( 'SONAR:')
        #  turtle_api.readSonar( direction, FOV, min_range, max_range, turtle_name)                      
        print( '\t {}',turtle_api.readSonar(0, math.pi/2, 0.5, 2,'turtle1'))
        print( 'COLOR:')
        print( '\t {}',color_api.check().r)
        print( '\t {}',color_api.check().g)
        print( '\t {}',color_api.check().b)
        print( turtle_api.setPen('turtle1',set_pen_req))
        print( turtle_api.getColisions(['turtle1', 'turtle2'], 1))
        cmd = Twist()
        cmd.linear.x = 0.0 
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0 # theta
        if turtle_api.hasTurtle('turtle2'):
            print( "CMD status:  {}", turtle_api.setVel('turtle2', cmd))
        print( '---------------------------------')
        frame_pixel_size = 200
        # camera in front
        x_offset = 0
        #camera from above
        # x_offset = -frame_pixel_size/2
        # show_matrix_cells_and_goal -- show matrix cells in turtle world
        show_matrix_cells_and_goal = True
        img_response = turtle_api.readCamera(name='turtle2', frame_pixel_size = frame_pixel_size, cell_count=16, x_offset=x_offset, \
                                        goal = Pose(x=10,y=5,theta=0), show_matrix_cells_and_goal=show_matrix_cells_and_goal)
        i = 0
        print( "Matrix: ")
        
        if VISUALIZE:
            r_matrix = np.zeros((len(img_response.m_rows[0].cells),len(img_response.m_rows)))
            r_row = np.zeros(len(img_response.m_rows[0].cells))
            g_matrix = np.zeros((len(img_response.m_rows[0].cells),len(img_response.m_rows)))
            g_row = np.zeros(len(img_response.m_rows[0].cells))
            b_matrix = np.zeros((len(img_response.m_rows[0].cells),len(img_response.m_rows)))
            b_row = np.zeros(len(img_response.m_rows[0].cells))
            dist_matrix = np.zeros((len(img_response.m_rows[0].cells),len(img_response.m_rows)))
            dist_row = np.zeros(len(img_response.m_rows[0].cells))
        # g_matrix = cv2.CreateMat((len(row.cells),len(img_response.m_rows)),no_of_bits,channels)
        # b_matrix = cv2.CreateMat((len(row.cells),len(img_response.m_rows)),no_of_bits,channels) 
        for row in img_response.m_rows:
            j = 0
            for cell in row.cells:
                r_row[j] = cell.red
                g_row[j] = cell.green
                b_row[j] = cell.blue
                print( "row:  {}", r_row)
                print( "\tCELL_"+str(i)+"_"+str(j)+":" )
                print( "\t\tR:  {}", cell.red)
                print( "\t\tG:  {}", cell.green)
                print( "\t\tB:  {}", cell.blue)
                print( "\t\tDist:  {}", cell.distance)
                j +=1
            r_matrix[i]=r_row
            g_matrix[i]=g_row
            b_matrix[i]=b_row
            i += 1

        if VISUALIZE:
            axR.imshow(r_matrix, norm=plt.Normalize(vmin=-1, vmax=1),cmap='hot')
            axG.imshow(g_matrix, norm=plt.Normalize(vmin=0, vmax=255),cmap='hot')
            axB.imshow(b_matrix, norm=plt.Normalize(vmin=-1, vmax=1),cmap='hot')
            # im_r = plt.imshow(r_matrix, norm=plt.Normalize(vmin=-1, vmax=1),cmap='hot')
            # im_g = plt.imshow(g_matrix, norm=plt.Normalize(vmin=0, vmax=255),cmap='hot')
            # im_b = plt.imshow(b_matrix, norm=plt.Normalize(vmin=-1, vmax=1),cmap='hot')
            # plt.colorbar(im, orientation='horizontal')
            plt.pause(0.001)
            plt.show(block=False)

        # cv_image = bridge.imgmsg_to_cv2(img_response.image, desired_encoding='passthrough')
        # cv2.imwrite('/tmp/cv_out.jpg', cv_image) 
        rate.sleep()
        k += 1 