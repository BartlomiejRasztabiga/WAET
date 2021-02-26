#!/usr/bin/env python
# encoding: utf8
import rospy
import math
import turtlesim
from turtlesim.msg import * # Pose
from turtlesim.srv import * #GetTurtles, GetPose, Spawn, GetSonar
# from turtlesim.msg import * 
from geometry_msgs.msg import Twist


class ColorSensor():
	def __init__(self, owner):
		isinstance(owner, str)
		self.owner = owner
		topic_name="/"+owner+"/color_sensor"
		rospy.Subscriber(topic_name, turtlesim.msg.Color, self.topic_callback)
	
	def topic_callback(self, data):
		self.colour = data

	def check(self):
		return self.colour

class TurtlesimSIU():
	"""docstr for TurtlesimSIU"""
	def __init__(self):
		required_services = ['spawn', 'get_turtles', 'get_pose', 'get_sonar']
		print "Waiting for services: ", required_services
		for name in required_services:
			rospy.wait_for_service(name)
			print "Connected to: ", name
		self.get_turtles = rospy.ServiceProxy('get_turtles', turtlesim.srv.GetTurtles)
		self.get_pose = rospy.ServiceProxy('get_pose', turtlesim.srv.GetPose)
		self.spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		self.get_sonar = rospy.ServiceProxy('get_sonar', turtlesim.srv.GetSonar)
		self.vel_publishers = []

	def getPose(self, turtle_name):
		isinstance(turtle_name, str)
		pose_result = self.get_pose(turtle_name)
		return pose_result.pose

	def setVel(self, turtle_name, vel):
		isinstance(turtle_name, str)
		isinstance(vel, Twist)
		for record in self.vel_publishers:
			if turtle_name == record['name']:
				record['publisher'].publish(vel)
				return True
		return False

	def setPen(self, turtle_name, req):
		isinstance(req, turtlesim.srv.SetPenRequest)
		isinstance(turtle_name, str)
		srv_name = '/'+turtle_name+'/set_pen'
		rospy.wait_for_service(srv_name)
		set_pen = rospy.ServiceProxy(srv_name, turtlesim.srv.SetPen)
		resp = set_pen(req)
		return resp

	def spawnTurtle(self, turtle_name, pose):
		isinstance(pose, turtlesim.msg.Pose)
		isinstance(turtle_name, str)
		try:
			vel_topic = '/'+turtle_name+'/cmd_vel'
			self.vel_publishers.append({'name': turtle_name, 'publisher': rospy.Publisher(vel_topic,Twist,queue_size=10)})
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

	def readCamera(self, owner):
		isinstance(owner, str)
		camera_result = self.get_camera_image(owner)
		return camera_result.image

	def getColisions(self, names, collision_range):
		for name in names:
			isinstance(name, str)
		turtles =[]
		for name in names:
			turtles.append({'name': name, 'pose': self.getPose(name)})
		# print 'turtles: ', turtles
		collisions=[]
		for main in turtles:
			for reference in turtles:
				# print "main ", main['name']
				# print "reference ", reference['name']
				dist = math.sqrt(pow(main['pose'].x - reference['pose'].x,2)+pow(main['pose'].y - reference['pose'].y,2))
				if dist > 0.01:
					print "dist: ", dist 
				if dist < collision_range:
					collision = {'name1':main['name'], 'name2':reference['name']}
					reverse_collision = {'name1':reference['name'], 'name2':main['name']}
					if not reverse_collision in collisions and reverse_collision != collision:
						collisions.append(collision)
		return collisions
			
	def readColor(self, owner):
		isinstance(fov_center, float)
		isinstance(fov_range, float)
		isinstance(range_min, float)
		isinstance(range_max, float)
		isinstance(owner, str)
		sonar_result = self.get_sonar(fov_center, fov_range, range_min, range_max, owner)
		return sonar_result.closes