#!/usr/bin/env python
# encoding: utf8
import rospy
import turtlesim
from turtlesim.srv import * #GetTurtles, GetPose, Spawn, GetSonar
from turtlesim.msg import *

class TurtlesimSIU():
	"""docstr for TurtlesimSIU"""
	def __init__(self):
		required_services = ['spawn', 'get_turtles', 'get_pose', 'get_sonar']
		print "Waiting for services: ", required_services
		for name in required_services:
			rospy.wait_for_service(name)
			print "Connected to: ", name
		self.get_turtles = rospy.ServiceProxy('get_turtles', GetTurtles)
		self.get_pose = rospy.ServiceProxy('get_pose', turtlesim.srv.GetPose)
		self.spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		self.get_sonar = rospy.ServiceProxy('get_sonar', turtlesim.srv.GetSonar)

	def getPose(self, turtle_name):
		isinstance(turtle_name, str)
		pose_result = self.get_pose(turtle_name)
		return pose_result.pose

	def spawnTurtle(self, turtle_name, pose):
		isinstance(pose, Pose)
		isinstance(turtle_name, str)
		try:
			spawn_result = self.spawn(x=pose.x, y=pose.y, theta=pose.theta, name=turtle_name)
			if spawn_result.name == turtle_name:
				return True
			else:
				return False
		except:
			raise Exception("Cannot spawn this turtle: <", turtle_name,"> :", pose)
	def readSonar(self, fov_center, fov_range, range_min, range_max, owner):
		isinstance(fov_center, float)
		isinstance(fov_range, float)
		isinstance(range_min, float)
		isinstance(range_max, float)
		isinstance(owner, str)
		sonar_result = self.get_sonar(fov_center, fov_range, range_min, range_max, owner)
		return sonar_result.closest