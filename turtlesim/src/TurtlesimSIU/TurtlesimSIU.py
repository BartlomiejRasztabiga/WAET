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
		"""! The ColorSensor class initializer.
        @param owner  The name of the turtle that owns the sensor.
        @return  An instance of the ColorSensor class initialized with the specified turtle name.
        """
		isinstance(owner, str)
		self.owner = owner
		topic_name="/"+owner+"/color_sensor"
		rospy.Subscriber(topic_name, turtlesim.msg.Color, self.topic_callback)
	
	def topic_callback(self, data):
		"""! Updates current color below the turtle. It is called each 16 milisecond
        """
		self.colour = data

	def check(self):
		"""! Returns last color received by the sensor.
        @return  last color received by the sensor. The returned object has 'r', 'g' and 'b' fields and their values are between 0-255.
        """
		return self.colour

class TurtlesimSIU():
	"""docstr for TurtlesimSIU"""
	def __init__(self):
		"""! The TurtlesimSIU class initializer. <b>It should be called AFTER the turtle environment startup</b>.
      	@return  An instance of the TurtlesimSIU class.
        """
		required_services = ['spawn', 'get_turtles', 'get_pose', 'get_sonar']
		print( "Waiting for services: {}", required_services)
		for name in required_services:
			rospy.wait_for_service(name)
			print( "Connected to:  {}", name)
		self.get_turtles = rospy.ServiceProxy('get_turtles', turtlesim.srv.GetTurtles)
		self.get_pose = rospy.ServiceProxy('get_pose', turtlesim.srv.GetPose)
		self.spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		self.get_sonar = rospy.ServiceProxy('get_sonar', turtlesim.srv.GetSonar)
		self.get_camera_image = rospy.ServiceProxy('get_camera_image', turtlesim.srv.GetCameraImage)
		self.has_turtle = rospy.ServiceProxy('has_turtle', turtlesim.srv.HasTurtle)
		self.kill_turtle = rospy.ServiceProxy('kill', turtlesim.srv.Kill)
		self.vel_publishers = []
		self.teleport_srvs = []

	def getPose(self, turtle_name):
		"""! Returns current pose of the given turtle.
        @param turtle_name  The name of the turtle.
        @return  is a Pose object : <br>
        float32 x<br> (unit: meter)
		float32 y<br> (unit: meter)
		float32 theta<br> (unit: radian)
		float32 linear_velocity<br> (unit: meter/sec)
		float32 angular_velocity <br> (unit: radian/sec)
        """
		isinstance(turtle_name, str)
		pose_result = self.get_pose(turtle_name)
		return pose_result.pose

	def setVel(self, turtle_name, vel):
		"""! Sets velocity to the given turtle.
        @param turtle_name  The name of the turtle.
        @param vel  geometry_msgs.msg.Twist object specifying the velocity.
        @return  True if the velocity was set
        """
		isinstance(turtle_name, str)
		isinstance(vel, Twist)
		for record in self.vel_publishers:
			if turtle_name == record['name']:
				record['publisher'].publish(vel)
				return True
		return False

	def setPen(self, turtle_name, req):
		"""! Sets the given turtle's pen.
        @param turtle_name  The name of the turtle.
        @param req  turtlesim.srv.SetPenRequest(r, g, b, width, off) object specifying the pen configuration.
        """
		isinstance(req, turtlesim.srv.SetPenRequest)
		isinstance(turtle_name, str)
		srv_name = '/'+turtle_name+'/set_pen'
		rospy.wait_for_service(srv_name)
		set_pen = rospy.ServiceProxy(srv_name, turtlesim.srv.SetPen)
		resp = set_pen(req)

	def hasTurtle(self, turtle_name):
		"""! Checks if the given turtle exists.
        @param turtle_name  The name of the turtle.
        @return  True if the turtle exists.
        """
		isinstance(turtle_name, str)
		return self.has_turtle(turtle_name).result

	def killTurtle(self, turtle_name):
		"""! Kill the given turtle and remove its velocity publisher and teleport service client.
        @param turtle_name  The name of the turtle.
        """
		isinstance(turtle_name, str)
		req = KillRequest()
		req.name = turtle_name
		print( self.kill_turtle(req))
		self.vel_publishers = [i for i in self.vel_publishers if not (self.vel_publishers['name'] == turtle_name)]
		self.teleport_srvs = [i for i in self.teleport_srvs if not (self.teleport_srvs['name'] == turtle_name)]

	def spawnTurtle(self, turtle_name, pose):
		"""! Spawns the given turtle in the given localisation.
        @param turtle_name  The name of the turtle.
        @param pose  The pose of the turtle given by turtlesim.msg.Pose(x,y,theta). 'x' and 'y' in meters, theta in radians.
        @return True if succeeded
        """
		isinstance(pose, turtlesim.msg.Pose)
		isinstance(turtle_name, str)
		try:
			vel_topic = '/'+turtle_name+'/cmd_vel'
			abs_srv = '/'+turtle_name+'/teleport_absolute'
			rel_srv = '/'+turtle_name+'/teleport_relative'
			self.vel_publishers.append({'name': turtle_name, 'publisher': rospy.Publisher(vel_topic,Twist,queue_size=10)})
			self.teleport_srvs.append({'name': turtle_name, 'absolute': rospy.ServiceProxy(abs_srv,turtlesim.srv.TeleportAbsolute),
															'relative': rospy.ServiceProxy(rel_srv,turtlesim.srv.TeleportRelative)})
			spawn_result = self.spawn(x=pose.x, y=pose.y, theta=pose.theta, name=turtle_name)
			if spawn_result.name == turtle_name:
				return True
			else:
				return False
		except:
			raise Exception("Cannot spawn this turtle: <", turtle_name,"> :", pose)
			
	def readSonar(self, fov_center, fov_range, range_min, range_max, owner):
		"""! Checks the closes turtle in the area given by the parameters.
        @param owner  The name of the turtle owning the sonar.
        @param fov_center  The direction of the sonar center.
        @param fov_range  The angle of the sonar's field of view.
        @param range_min  The min range of the sonar.
        @param range_max  The max range of the sonar.
        @return distance to the closes turtle in the given area
        """
		isinstance(fov_center, float)
		isinstance(fov_range, float)
		isinstance(range_min, float)
		isinstance(range_max, float)
		isinstance(owner, str)
		sonar_result = self.get_sonar(fov_center, fov_range, range_min, range_max, owner)
		return sonar_result.closest

	def readCamera(self, name='turtle1', frame_pixel_size = 200, cell_count=16, x_offset=0,\
											goal=Pose(), show_matrix_cells_and_goal=False):
		"""! Reads image from the given turtles camera. The camera localisation and sensor size is configurable in the arguments. 
        @param name  The name of the turtle owning the camera.
        @param frame_pixel_size  The size of the camera sensor in pixels. The sensor is a square which a=frame_pixel_size.
        @param cell_count  The count of the returned matrix cells. The matrix is square and is divided into the given number of cells.
        @param x_offset  The offset in x direction (turtle front) of the camera localisation. If equals 0, the camera is in front of the turtle, and if equals -frame_pixel_size/2, the turtle is in the image center.
        @param goal  The goal of the turtle to calculate distance from each cell to the goal. It is given by turtlesim.msg.Pose(x,y,theta). 'x' and 'y' in meters, theta in radians.
        @param show_matrix_cells_and_goal Triggers visualisation of the cells, the goal and the turtle pose 
        @return The NxN matrix, and each cell of the matrix has 4 fields: cell.red, cell.green, cell.blue, cell.distance. The latter is the distance to the specified goal.
        """
		isinstance(name, str)
		isinstance(frame_pixel_size, int)
		isinstance(cell_count, int)
		isinstance(x_offset, int)
		isinstance(goal, Pose)
		isinstance(show_matrix_cells_and_goal, bool)
		return self.get_camera_image(name, frame_pixel_size, cell_count, x_offset, goal,show_matrix_cells_and_goal)

	def getColisions(self, names, collision_range):
		"""! Check collistions between the given turtles.
        @param names  The names of the turtles.
        @param collision_range  The minimal distance between the turtles.
        @return list of the turtle pairs that colide
        """
		for name in names:
			isinstance(name, str)
		turtles =[]
		for name in names:
			turtles.append({'name': name, 'pose': self.getPose(name)})
		# print( 'turtles: ', turtles
		collisions=[]
		for main in turtles:
			for reference in turtles:
				# print( "main ", main['name']
				# print( "reference ", reference['name']
				dist = math.sqrt(pow(main['pose'].x - reference['pose'].x,2)+pow(main['pose'].y - reference['pose'].y,2))
				if dist > 0.01:
					print( "dist:  {}", dist )
				if dist < collision_range:
					collision = {'name1':main['name'], 'name2':reference['name']}
					reverse_collision = {'name1':reference['name'], 'name2':main['name']}
					if not reverse_collision in collisions and reverse_collision != collision:
						collisions.append(collision)
		return collisions
	
	def setPose(self, turtle_name, pose, mode='absolute'):
		"""! Teleport the given turtle.
        @param turtle_name  The turtle_name of the turtle.
        @param pose  The destination pose of the turtle.
        @param mode the mode of the teleportaiton ('absolute', 'relative'). <b>For 'absolute' translate and rotate afterwards, and for 'relative' rotate and translate afterwards </b>
        @return True if succeeded
        """
		isinstance(turtle_name, str)
		isinstance(pose, Pose)
		if mode not in ['absolute','relative']:
			return False
		elif mode == 'absolute':
			req = turtlesim.srv.TeleportAbsoluteRequest()
			req.x = pose.x
			req.y = pose.y
			req.theta = pose.theta
		elif mode == 'relative':
			req = turtlesim.srv.TeleportRelativeRequest()
			req.linear = pose.x
			req.angular = pose.theta
		for record in self.teleport_srvs:
			if turtle_name == record['name']:
				record[mode](req)
				return True
		return False

	def pixelsToScale(self):
		"""! Returns the pixels/meter scaling factor.
		@return The pixels/meter scaling factor.
        """
		return rospy.get_param("/pixels_meter_scale")
