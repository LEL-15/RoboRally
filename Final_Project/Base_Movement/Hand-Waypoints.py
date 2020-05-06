#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState
from PIL import Image
from std_msgs.msg import UInt16
import json

import sys, select, termios, tty

state = 0
pose_x = 0
pose_y = 0
pose_theta = 0
#Needed map variables
g_MAP_SIZE_X = 8 # 2m wide
g_MAP_SIZE_Y = 8 # 1.5m tall
g_MAP_RESOLUTION_X = .1 #Each column 1 meter
g_MAP_RESOLUTION_Y = .1 #Each row 1 meter
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map
  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map
  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (j+0.5)*g_MAP_RESOLUTION_X -4, (i+0.5)*g_MAP_RESOLUTION_Y - 4

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map
  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int((y+4) / g_MAP_RESOLUTION_Y), int((x+4) / g_MAP_RESOLUTION_X)

def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  #Load image
  global MAP_SIZE_X, MAP_SIZE_Y, g_WORLD_MAP

  if img_filename is None:
      grid = np.zeros([MAP_SIZE_X,MAP_SIZE_Y])
      print("Big Problem")
      return

  img = Image.open(img_filename)
  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height

  grid = np.zeros([MAP_SIZE_Y, MAP_SIZE_X])
  for y in range(MAP_SIZE_Y):
      for x in range(MAP_SIZE_X):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel # Dark pixels have high values to indicate being occupied/having something interesting

  #Detecting obstacles from image phase
  obstacle = False
  g_WORLD_MAP = []
  #Generate an array of 0's
  for i in range(g_NUM_Y_CELLS):
    for j in range(g_NUM_X_CELLS):
      g_WORLD_MAP.append(0)
  #loops through intensity matrix, if any part of the section has an obstacle, it is considered blocked
  multiplier = float(g_NUM_Y_CELLS)/float(MAP_SIZE_Y)
  multiplier = multiplier
  for y in range(MAP_SIZE_Y):
    for x in range(MAP_SIZE_X):
      obstacle = False
      if grid[y][x] >= 254:
        obstacle = True
      if obstacle == True:
        loc = (ij_to_vertex_index(int(x*multiplier), int(y*multiplier)))
        if(loc > (g_NUM_Y_CELLS**2) - 1):
          loc = (g_NUM_Y_CELLS**2) - 1
        g_WORLD_MAP[loc] = 1
  for i in range(6):
    add_padding()
def add_padding():
  #Add padding to obstacles
  global MAP_SIZE_X, MAP_SIZE_Y, g_WORLD_MAP

  g_HOLD_MAP = []
  for i in range(g_NUM_Y_CELLS):
    for j in range(g_NUM_X_CELLS):
      g_HOLD_MAP.append(0)

  #For every cell, see if ther is adjacent wall, and if so make also wall
  for y in range(g_NUM_Y_CELLS):
    for x in range(g_NUM_X_CELLS):
      obstacle = False
      size = g_NUM_X_CELLS * g_NUM_Y_CELLS
      local1 = ij_to_vertex_index(x+1, y)
      local2 = ij_to_vertex_index(x-1, y)
      local3 = ij_to_vertex_index(x, y+1)
      local4 = ij_to_vertex_index(x, y-1)
      if(local1 < size):
        if(g_WORLD_MAP[local1] == 1):
          obstacle = True
      if(local2 >= 0):
        if(g_WORLD_MAP[local2] == 1):
          obstacle = True
      if(local3 < size):
        if(g_WORLD_MAP[local3] == 1):
          obstacle = True
      if(local4 >= 0):
        if(g_WORLD_MAP[local4] == 1):
          obstacle = True
      if(obstacle):
        local = ij_to_vertex_index(x, y)
        g_HOLD_MAP[local] = 1
  for index in range(len(g_WORLD_MAP)):
    if(g_HOLD_MAP[index] == 1):
      g_WORLD_MAP[index] = 1

def render_map(map_array):

  global g_NUM_X_CELLS, g_NUM_Y_CELLS
  global g_WORLD_MAP
  '''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells
    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .
    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  #lines = [["" for i in range(g_NUM_X_CELLS)] for j in range(g_NUM_Y_CELLS)]
  #line = ""
  for i in range(g_NUM_Y_CELLS):
    for j in range(g_NUM_X_CELLS):
      coordinate = ij_to_vertex_index(j, i)
      if map_array[ij_to_vertex_index(j,i)] == 0 or map_array[ij_to_vertex_index(j,i)] == -1:
        print(" . ", end = ''),
      #Obstacle
      elif map_array[ij_to_vertex_index(j,i)] == 1:
        print("[ ]", end = ''),
      elif map_array[ij_to_vertex_index(j,i)] == 2:
        print(" S ", end = ''),
      elif map_array[ij_to_vertex_index(j,i)] == 3:
        print(" D ", end = ''),
      else:
        print("[ ]", end = ''),
    print('\n')

def callback_update_odometry():
    global pose_x, pose_y, pose_theta
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    object_coordinates = model_coordinates("fetch", "")
    pose_x = object_coordinates.pose.position.x
    pose_y = object_coordinates.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([object_coordinates.pose.orientation.x, object_coordinates.pose.orientation.y, object_coordinates.pose.orientation.z, object_coordinates.pose.orientation.w])
    pose_theta = correctTheta(yaw)

def updateDistance(pose_x, pose_y, dest_pose_x, dest_pose_y):
  return math.sqrt(pow(abs(pose_x - dest_pose_x), 2)+ pow(abs(pose_y - dest_pose_y), 2));

def correctTheta(raw):
    if(raw < 0):
        raw = 2*math.pi + raw
    return raw

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)
      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  global g_NUM_Y_CELLS, g_NUM_X_CELLS
  global g_WORLD_MAP
  map_size = g_NUM_Y_CELLS * g_NUM_X_CELLS
  #Check vertices not out of bounds
  if vertex_source < len(g_WORLD_MAP) and vertex_dest < len(g_WORLD_MAP):
    srcx, srcy = vertex_index_to_ij(vertex_source)
    destx, desty = vertex_index_to_ij(vertex_dest)
    dist = abs(srcx - destx) + abs(srcy - desty)

    if dist == 1 and g_WORLD_MAP[vertex_source] != 1 and g_WORLD_MAP[vertex_dest] != 1:
      return 1
  return 1000


def run_dijkstra(source_vertex):
  '''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run
  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
  '''
  global g_NUM_X_CELLS, g_NUM_Y_CELLS
  global g_WORLD_MAP
  
  # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
  dist = [float("inf")] * g_NUM_X_CELLS * g_NUM_Y_CELLS
  prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS
  # Queue for identifying which vertices are up to still be explored:
  # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
  Q_cost = [(v, get_travel_cost(source_vertex, v)) for v in range(g_NUM_X_CELLS*g_NUM_Y_CELLS)]
  Q_cost[source_vertex] = (source_vertex, 0)

  while(Q_cost):
    v = min(Q_cost,key=lambda t: t[1])
    dist[v[0]] = v[1]
    n = []
    x,y = vertex_index_to_ij(v[0])
    n.append(ij_to_vertex_index(x,y+1))
    n.append(ij_to_vertex_index(x,y-1))
    n.append(ij_to_vertex_index(x+1,y))
    n.append(ij_to_vertex_index(x-1,y))
    Q_cost.remove(v)
    for u in n:
      for i in Q_cost:
        if u == i[0]:
          val = get_travel_cost(v[0],u) + v[1]
          if val < dist[u] and val < 500:
            dist[u] = val
            prev[u] = v[0]
            Q_cost.remove(i)
            Q_cost.insert(0,(u,dist[u]))

  ratio = 1200/g_NUM_X_CELLS
  coords = []
  for i in range(len(prev)):
    if prev[i] != -1:
      tmp = vertex_index_to_ij(prev[i])
      tmp = ij_coordinates_to_xy_coordinates(tmp[0],tmp[1])
      coord = (tmp[0], tmp[1])
      coords.append(coord)
    else:
      coords.append(-1)
  return prev, coords


def reconstruct_path(prev, coords, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = []
  final_coords = []
  current = dest_vertex
  while current != source_vertex:
    if current == -1:
      return []
    else:
      final_path.insert(0,current)
      final_coords.insert(0, coords[current])
      current = prev[current]

  final_path.insert(0,source_vertex)
  return final_path, final_coords

def render_map_path(map_array, path_array):

  global g_NUM_X_CELLS, g_NUM_Y_CELLS
  global g_WORLD_MAP
  '''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells
    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .
    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  #lines = [["" for i in range(g_NUM_X_CELLS)] for j in range(g_NUM_Y_CELLS)]
  #line = ""
  for i in range(g_NUM_Y_CELLS):
    for j in range(g_NUM_X_CELLS):
      coordinate = ij_to_vertex_index(j, i)
      if coordinate in path_array:
        print(" x ", end = ''),
      elif map_array[ij_to_vertex_index(j,i)] == 0:
        print(" . ", end = ''),
      elif map_array[ij_to_vertex_index(j,i)] == 1:
        print("[ ]", end = ''),
      elif map_array[ij_to_vertex_index(j,i)] == 2:
        print(" S ", end = ''),
      elif map_array[ij_to_vertex_index(j,i)] == 3:
        print(" D ", end = ''),
    print('\n')

def goTo(dest_pose_x, dest_pose_y):
    global pose_x, pose_y, pose_theta, pub
    rate = rospy.Rate(1.0/.4)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    distance = updateDistance(pose_x, pose_y, dest_pose_x, dest_pose_y)
    twist = Twist()
    while (distance > .075):
        use = (dest_pose_y - pose_y) / (dest_pose_x - pose_x)
        goal_angle = correctTheta(math.atan2(dest_pose_y - pose_y, dest_pose_x -pose_x))
        while(abs(goal_angle - pose_theta) > math.pi/50):
          #Spin
          #Default spin is to right
          spin = -1
          opposite = pose_theta + math.pi
          #See if should spin to left
          if(pose_theta < math.pi):
            if(goal_angle > pose_theta and goal_angle < opposite):
                spin = 1
          else:
            if((goal_angle > pose_theta and goal_angle < 2*math.pi) or (goal_angle > 0 and goal_angle +2*math.pi < opposite)):
                spin = 1
          twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
          twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = spin;
          pub.publish(twist)
          #Wait
          rate.sleep()
          #Stop spin
          twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
          twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
          pub.publish(twist)
          callback_update_odometry()
        #Go Forward
        twist.linear.x = .5; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
        pub.publish(twist)
        #Wait
        rate.sleep()
        #Stop
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
        pub.publish(twist)
        #Update distance
        callback_update_odometry()
        distance = updateDistance(pose_x, pose_y, dest_pose_x, dest_pose_y)
    return

def callback_update_state(data):
  global state
  #DONE: Load data into your program's local state variables
  state = data.data
  print(state)


if __name__=="__main__":
    subscriber_state = rospy.Subscriber('/state', UInt16, callback_update_state)
    publisher_state = rospy.Publisher('/state', UInt16, queue_size = 10)
    callback_update_odometry()
    rospy.init_node('teleop_twist_keyboard')
    _load_img_to_intensity_matrix('equalmap.png')
    #Waypoints to navigate to
    loc = [[(3.25, -2), (3.25, -3), (2.5, -3), (2.25, -3)],
           [(3.25, -2), (0, 0)],
           [(2, 2.75), (3, 2.75), (3, 2.25), (3, 1.75)], 
           [(2, 2.75), (0, 0)],
           [(-3, 1.7), (-3, 3), (-2.5, 3), (-2.25, 3)],
           [(-3, 1.7), (0, 0)],
           [(-1.7, -3), (-3, -3), (-3, -2.5), (-3, -2.25)],
           [(-1.7, -3), (0, 0)]
          ]
    rate = rospy.Rate(1.0/.4)
    #Keeping track of what path we are on
    index = 0
    change = 1
    # #While searching with camera off or on
    # while(state == 0 or state  == 1 or state == 6):
    #   coords_path = loc[index]
    #   #Iterate over path
    #   for waypoint in coords_path:
    #     #If we are analyzing a picture, wait
    #     while(state == 1 or state == 6):
    #       rate.sleep()
    #     # If we see the color we want, switch to state 3 and move arm
    #     if (state == 2):
    #       publisher_state.publish(3)
    #       state = UInt16(3)
    #     while(state == 3):
    #       rate.sleep()
    #     goTo(waypoint[0], waypoint[1])
    #     #When driving at object
    #     if(loc.index(coords_path) %2 == 0 and coords_path.index(waypoint) == 2):
    #       publisher_state.publish(1)

    #   # Wait for robot to analyze picture
    #   while(state == 6):
    #     rate.sleep() 

    #   #If reached table and object not seen, turn off camera
    #   if(state != 2):
    #     publisher_state.publish(0)
    #   index+=change
    #   if(index >= len(loc)):
    #       index = 0

    # if (state == 2 or state == 3):
    #   rate.sleep()
    # #Once are retreived, return to center
    # coords_path = loc[index]
    # for waypoint in coords_path:
    #   goTo(waypoint[0], waypoint[1])
    # index+=change

    working = True

    while (working):
      if state == 0:
        #search
        coords_path = loc[index]
        #Iterate over path
        for waypoint in coords_path:
          goTo(waypoint[0], waypoint[1])
          #When driving at object
          if(loc.index(coords_path) %2 == 0 and coords_path.index(waypoint) == 2):
            publisher_state.publish(1)
            state = UInt16(1)
            index += change
            break
        if(index >= len(loc)):
          index = 0

      elif state == 1:
        #wait for objectDetection to take picture
        rate.sleep()
      elif state == 2:
        #switch to 3 and call arm
        publisher_state.publish(UInt16(3))
        state = UInt16(3)        
      elif state == 3:
        #wait for arm
        rate.sleep()
      elif state == 4:
        #move to box and call arm
        coords_path = loc[index-change]
        goTo(coords_path[3][0], coords_path[3][1])
        goTo(coords_path[2][0], coords_path[2][1])
        publisher_state.publish(UInt16(5))  
        state = 5
      elif state == 5:      
        #return the arm
        rate.sleep()       
      elif state == 6:
        #wait for robot to analyze picture
        rate.sleep()
      elif state == 7:      
        #return home
        coords_path = loc[index]
        for waypoint in coords_path:
          goTo(waypoint[0], waypoint[1])
        index+=change 
        working = False        