#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import copy 

global turtle_map
turtle_map = {"names": [], "poses": []}
 
DEBUG=False
def logDEBUG(input_string):
  if DEBUG:
    logDEBUG( input_string)

def hasInNames(names, name):
  for y in names:
    if y == name:
      return True
  return False
def getTurtlesByRange(max_range,min_range, turtles, reference):
  ref_id = turtles["names"].index(reference)
  i =0
  dist_map=[]
  for record in turtles["poses"]:
    logDEBUG( "poses: "+str(turtles["poses"]))
    dist = math.sqrt(math.pow(record.x - turtles["poses"][ref_id].x, 2)+math.pow(record.y - turtles["poses"][ref_id].y, 2))
    logDEBUG( "dist: "+str(dist))
    logDEBUG( "min_range: "+str(min_range))
    logDEBUG( "max_range: "+str(max_range))
    if dist >= min_range and dist <= max_range:
      dist_map.append({'name': turtles["names"][i],'id': i, 'distance': math.sqrt(math.pow(record.x - turtles["poses"][ref_id].x, 2)+math.pow(record.y - turtles["poses"][ref_id].y, 2))
                       , 'x': record.x, 'y':record.y})
    i= i +1
  return dist_map

def inDirection(fov_range, fov_center, in_range, reference):
  global turtle_map
  ref_id = turtle_map["names"].index(reference)
  logDEBUG( "ref_id= "+str(ref_id))
  found_list=[]
  for record in in_range:
    diff_y = record['y']-turtle_map["poses"][ref_id].y
    diff_x = record['x']-turtle_map["poses"][ref_id].x
    direction = math.atan2(diff_y, diff_x)- turtle_map["poses"][ref_id].theta 
    if(direction > math.pi):
      direction -= 2*math.pi
    if(direction < -math.pi):
      direction += 2*math.pi
    logDEBUG( "direction: "+str(direction))
    if direction >= fov_center-fov_range and direction <= fov_center+fov_range:
      logDEBUG( "found!: "+str(record['name'] ))
      found_list.append(record)
  return found_list

def sonar_callback(req):
  global turtle_map
  response = turtlesim.srv.GetSonarResponse()
  in_range = getTurtlesByRange(max_range=req.range_max, min_range=req.range_min,turtles=turtle_map, reference=req.name)
  logDEBUG( in_range)
  found_list = inDirection(req.fov_range, req.fov_center, in_range, req.name)
  closest = None
  log = "found_list, "+ str(found_list)
  logDEBUG( log)
  if len(found_list) > 0:
    for record in found_list:
      logDEBUG( found_list)
      if closest == None:
        closest = record
      elif record['distance'] < closest['distance']:
        closest = record
    response.closest = closest['distance']
  else:
    response.closest = -1
  return response


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    rospy.wait_for_service('spawn')
    get_turtles = rospy.ServiceProxy('get_turtles', turtlesim.srv.GetTurtles)
    get_pose = rospy.ServiceProxy('get_pose', turtlesim.srv.GetPose)
    sonar_srv = rospy.Service('get_sonar', turtlesim.srv.GetSonar, sonar_callback)
    turtle_list = []
    rate = rospy.Rate(10.0)
    global turtle_map

    while not rospy.is_shutdown():
      try:
          turtle_list = get_turtles()
          for name in turtle_list.list:
            if not hasInNames(turtle_map["names"], name):
              pose_req = turtlesim.srv.GetPoseRequest()
              pose_req.name = name
              turtle_map["names"].append(name)
              turtle_map["poses"].append(get_pose(pose_req).pose)
              logDEBUG( turtle_map)
            else:
              turtle_map["poses"][turtle_map["names"].index(name)]=get_pose(name).pose

          i = 0
          for record in turtle_map["names"]:
            if record not in turtle_list.list:
              logDEBUG( "remove: "+str(record))
              rm_id = turtle_map["names"].index(record)
              del turtle_map["names"][rm_id]
              del turtle_map["poses"][rm_id]
      except:
          continue
      rate.sleep()