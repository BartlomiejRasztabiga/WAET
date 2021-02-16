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
    print "poses: ",turtles["poses"]
    dist = math.sqrt(math.pow(record.x - turtles["poses"][ref_id].x, 2)+math.pow(record.y - turtles["poses"][ref_id].y, 2))
    print "dist: ",dist
    if dist >= min_range and dist <= max_range:
      dist_map.append({'name': turtles["names"][i],'id': i, 'distance': math.sqrt(math.pow(record.x - turtles["poses"][ref_id].x, 2)+math.pow(record.y - turtles["poses"][ref_id].y, 2))
                       , 'x': record.x, 'y':record.y})
    i= i +1
  return dist_map

def inDirection(fov_range, fov_center, in_range, reference):
  global turtle_map
  ref_id = turtle_map["names"].index(reference)
  print "ref_id= ", ref_id
  found_list=[]
  for record in in_range:
    diff_y = record['y']-turtle_map["poses"][ref_id].y
    diff_x = record['x']-turtle_map["poses"][ref_id].x
    direction = math.atan2(diff_y, diff_x)- turtle_map["poses"][ref_id].theta 
    if(direction > math.pi):
      direction -= 2*math.pi
    if(direction < -math.pi):
      direction += 2*math.pi
    print "direction: ", direction
    if direction >= fov_center-fov_range and direction <= fov_center+fov_range:
      print "found!: ", record['name'] 
      found_list.append(record)
  return found_list

def sonar_callback(req):
  global turtle_map
  response = turtlesim.srv.GetSonarResponse()
  in_range = getTurtlesByRange(max_range=req.range_min, min_range=req.range_max,turtles=turtle_map, reference=req.name)
  print in_range
  found_list = inDirection(req.fov_range, req.fov_center, in_range, req.name)
  closest = None
  print "found_list, ", found_list
  if len(found_list) > 0:
    for record in found_list:
      print found_list
      if closest == None:
        closest = record
      elif record['distance'] < closest['distance']:
        closest = record
    response.closest = closest['distance']
  else:
    response.closest = -1
  return response


  # response = turtlesim.srv.GetSonarResponse()
  # req.fov
  # req.range
  # req.name 
  # map_ = copy.copy(turtle_map)

  # for record in map_["names"]:
  #   if record == req.name:
  #     rm_id = turtle_map["names"].index(record)
  #     del turtle_map["names"][rm_id]
  #     del turtle_map["poses"][rm_id]
  
  #     print "remove: ", record
  #     rm_id = turtle_map["names"].index(record)
  #     del turtle_map["names"][rm_id]
  #     del turtle_map["poses"][rm_id]

  # for record in turtle_map:
  #   rel_x = record["pose"].x + turtle_map
  #   rel_y
  #   rel_pose = {"name": record["name"], "pose": get_pose(pose_req).pose}
  # return response

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
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
              print turtle_map
            else:
              # print "IDX: ", turtle_map["names"].index(name)
              # print "POSE: ", get_pose(name).pose
              turtle_map["poses"][turtle_map["names"].index(name)]=get_pose(name).pose

          i = 0
          for record in turtle_map["names"]:
            if record not in turtle_list.list:
              print "remove: ", record
              rm_id = turtle_map["names"].index(record)
              del turtle_map["names"][rm_id]
              del turtle_map["poses"][rm_id]
          # print turtle_map
      except:
          continue
      rate.sleep()



# #!/usr/bin/env python
# # encoding: utf8

# import rospkg
# import sys
# import copy
# import time 

# import rospy
# from std_msgs.msg import String
# from turtlesim.msg import Pose

# class Sonar():
#     def __init__(self, sonar_fov, sonar_range, center):
#       isinstance(sonar_range, float)
#       isinstance(sonar_fov, float)
#       isinstance(center, Pose)
#       self.fov = sonar_fov
#       self.range = sonar_range

#     def detect(self):

# def check_sonar(sonar_fov, sonar_range, pose):
#     isinstance(sonar_range, float)
#     isinstance(sonar_fov, float)
#     isinstance(center, Pose)



# def main():
#     rospy.init_node('sonar_emulation')
#     rospy.sleep(0.5)
#     create_sonar = rospy.Service('create_sonar', AddTwoInts, create_sonar_callback)

#     pub = rospy.Service('/velma_cmd', roko_msgs.msg.UserRequest, queue_size=10)
#     intent = None
#     in_value = None
#     now = None
#     priority = 1
#     for idx in range(1, len(sys.argv), 2):
#         if sys.argv[idx] == 'intent':
#             intent = sys.argv[idx+1]
#         if sys.argv[idx] == 'in':
#             in_value = sys.argv[idx+1]
#         if sys.argv[idx] == 'now':
#             now = sys.argv[idx+1]
#         if sys.argv[idx] == 'priority':
#             priority = sys.argv[idx+1]
#     if intent is None:
#         raise Exception('Argument "intent" is missing in argv: ' + str(sys.argv))  
#     elif intent != "BG":
#         raise Exception('Only <BG> intent is available: ' + str(sys.argv))  

#     pub = rospy.Publisher('/velma_cmd', roko_msgs.msg.UserRequest, queue_size=10)
#     req = roko_msgs.msg.UserRequest()
#     req.intent_name = intent
#     req_time = datetime.now()
#     print "req_time: ", req_time
#     req.req_datetime = req_time.strftime("%d/%m/%Y, %H:%M:%S.%f")
#     req.param_names=['przedmiot']
#     req.param_values=['piwo']
#     req.priority = int(priority)
#     req.shdl_rules.rule_type = 'at'
#     if in_value is not None:
#         req_time += timedelta(minutes=int(in_value))
#     elif now is not None:
#         pass
#     req.shdl_rules.rule_value = req_time.strftime("%d/%m/%Y, %H:%M:%S.%f")
#     print  req.shdl_rules.rule_value
#     rospy.sleep(0.5)
#     pub.publish(req)
    

# if __name__ == '__main__':
#     main()