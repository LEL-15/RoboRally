import copy
import math
import random
import argparse
from PIL import Image
import numpy as np
from pprint import pprint
import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


#Lab 4 globals
pose2d_sparki_odometry = None 
#ATTEMPTED: Create data structure to hold map representation
world_map = [];

testMap = []

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None
g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 1.5 # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.5 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)


def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells)
    new_map[random_cell] = 1

  return new_map


def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
      grid = np.zeros([800,1200])
      return grid

  img = Image.open(img_filename)

  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height

  grid = np.zeros([img.height, img.width])
  for y in range(img.height):
      for x in range(img.width):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

  return grid


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
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map
  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(i // g_MAP_RESOLUTION_X), int(j // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

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
  global g_NUM_Y_CELLS, g_NUM_X_CELLS, g_WORLD_MAP
  map_size = g_NUM_Y_CELLS * g_NUM_X_CELLS
  answer = 1000
  #Check vertices not out of bounds
  """
  if(not(vertex_source < 0 or vertex_dest < 0 or vertex_source >= map_size or vertex_dest >= map_size)):
    source_x, source_y = vertex_index_to_ij(vertex_source)
    destination_x, destination_y = vertex_index_to_ij(vertex_dest)    
    #Check neither vetex occupied
    if(not(g_WORLD_MAP[vertex_source] == 1 or g_WORLD_MAP[vertex_dest] == 1) ):
      #If in same row and off by a column
      if(source_x == destination_x and (source_y == destination_y + 1 or source_y == destination_y-1)):
        answer = 1
      #If in same column and off by a row
      elif(source_y == destination_y and (source_x == destination_x + 1 or source_x == destination_x-1)):
        answer = 1
  """
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
          if val < dist[u]:
            dist[u] = val
            prev[u] = v[0]
            Q_cost.remove(i)
            Q_cost.insert(0,(u,dist[u]))

  ratio = 1200/g_NUM_X_CELLS
  coords = []
  for i in range(len(prev)):
    if prev[i] != -1:
      tmp = vertex_index_to_ij(prev[i])
      if not tmp[1] == (g_NUM_X_CELLS - 1) and not tmp[0] == (g_NUM_Y_CELLS - 1): 
        coord = ((tmp[0]*ratio + ratio/2)/666.66666,(tmp[1]*ratio + ratio/2)/666.66666)
        coords.append(coord)
      else:
        coord = ((tmp[0]*ratio - ratio/2)/666.66666,(tmp[1]*ratio - ratio/2)/666.66666)
        coords.append(coord)
    else:
      coords.append(-1)

  return prev, coords


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''

  # TODO: Insert your code here
  print(source_vertex)
  print(dest_vertex)
  final_path = []
  current = dest_vertex
  while current != source_vertex:
    if current == -1:
      return []
    else:
      final_path.insert(0,current)
      #print(prev)
      #print(current)
      current = prev[current]

  final_path.insert(0,source_vertex)
  # TODO: Insert your code here

  #print(final_path)
  return final_path



def render_map(map_array, path_array):

  global g_NUM_X_CELLS, g_NUM_Y_CELLS
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
  for j in range(g_NUM_Y_CELLS):
    for i in range(g_NUM_X_CELLS):
      #pint("")
      coordinate = ij_to_vertex_index(i, j)
      if coordinate in path_array:
        print(" x "),
      elif g_WORLD_MAP[coordinate] == 0:
        print(" . "),
      elif g_WORLD_MAP[coordinate] == 1:
        print("[ ]"),
    print('\n')


def part_1():
  global g_WORLD_MAP

  global g_dest_coordinates
  global g_src_coordinates
  global g_NUM_X_CELLS, g_NUM_Y_CELLS

  # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles

  g_WORLD_MAP = [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]
  g_NUM_X_CELLS = 4
  g_NUM_Y_CELLS = 4

  # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path
  print("Source: (" + str(g_src_coordinates[0]) + ", " + str(g_src_coordinates[1]) + ")" )
  print("Goal: (" + str(g_dest_coordinates[0]) + ", " + str(g_dest_coordinates[1]) + ")" )

  source_int = ij_to_vertex_index(g_src_coordinates[0], g_src_coordinates[1])
  dest_int = ij_to_vertex_index(g_dest_coordinates[0], g_dest_coordinates[1])
  path = reconstruct_path(run_dijkstra(source_int), source_int, dest_int)
  # Use render_map to render your initialized obstacle map
  render_map(g_WORLD_MAP, path)
  
  '''
  TODO-
    Display the final path in the following format:
    Source: (0,0)
    Goal: (3,1)
    0 -> 1 -> 2 -> 6 -> 7
  '''
  if(path != []):
    if(len(path) > 1):
      for i in range(len(path)):
        if i != len(path) - 1:
          print(str(path[i]) + " -> "),
        else:
          print(str(path[i])),
    else:
      print(path[0])
  else:
    print("ERROR: NO POSSIBLE PATH")

def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP
  global g_NUM_X_CELLS
  global g_NUM_Y_CELLS

  g_src_coordinates = (args.src_coordinates[0], str(1.2 - float(args.src_coordinates[1])))
  g_dest_coordinates = (args.dest_coordinates[0], str(1.2 - float(args.dest_coordinates[1])))

  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)
  x = np.arange(0,len(pixel_grid),40)
  y = np.arange(0,len(pixel_grid[0]),40)
  obstacle = False
  g_NUM_X_CELLS = 30
  g_NUM_Y_CELLS = 20
  g_WORLD_MAP = [0]*g_NUM_X_CELLS*g_NUM_Y_CELLS
  for i in x:
    for j in y:
      obstacle = False
      for k in range(i,i+40):
        for l in range(j,j+40):
          if pixel_grid[k][l] == 255:
            obstacle = True
      if obstacle == True:
        loc = ij_to_vertex_index(i/40,j/40)
        g_WORLD_MAP[loc] = 1

  print("\nSource: (" + str(g_src_coordinates[0]) + ", " + str(1.2 - float(g_src_coordinates[1])) + ")" )
  print("Goal: (" + str(g_dest_coordinates[0]) + ", " + str(1.2 - float(g_dest_coordinates[1])) + ")" )
  src = ij_to_vertex_index(math.floor(16.6667*float(g_src_coordinates[0])), math.floor(16.6667*float(g_src_coordinates[1])))
  dest = ij_to_vertex_index(math.floor(16.6667*float(g_dest_coordinates[0])), math.floor(16.6667*float(g_dest_coordinates[1])))
  path = reconstruct_path(run_dijkstra(int(src)), int(src), int(dest))
  render_map(g_WORLD_MAP, path)
  if(path != []):
    if(len(path) > 1):
      for i in range(len(path)):
        if i != len(path) - 1:
          print(str(path[i]) + " -> "),
        else:
          print(str(path[i])),
    else:
      print(path[0])
  else:
    print("ERROR: NO POSSIBLE PATH")

def init():
    #DONE: Set up your publishers and subscribers
    rospy.init_node("main", anonymous=True)
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global theta, value_array, ping_distance
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)


    #DONE: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()
    #DONE: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    msg = Int16(90)
    publisher_servo.publish(msg)
    #ATTEMPTED: Set map values to all be empty
    for i in range(20):
        temp = []
        for j in range(30):
            temp.append(0)
        testMap.append(temp)
    for i in range(280):
        world_map.append(0);
def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    #DONE: Copy this data into your local odometry variable
    global pose2d_sparki_odometry 
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta
    

def callback_update_state(data):
    global servo, value_array, ping_distance
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #DONE: Load data into your program's local state variables
    servo = state_dict["servo"]
    value_array = state_dict["light_sensors"]
    #Update map
    if('ping' in state_dict and state_dict['ping'] != -1):
        ping_distance = state_dict["ping"]
        populate_map_from_ping(ping_distance)

def lab_6(args):
  #Set everything up
  global publisher_odom, pose2d_sparki_odometry
  global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP
  publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
  publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
  init()
  g_src_coordinates = (args.src_coordinates[0], 1.2 - float(args.src_coordinates[1]))
  g_dest_coordinates = (args.dest_coordinates[0], 1.2 - float(args.dest_coordinates[1]))
  
  start = Pose2D();
  start.x = g_src_coordinates[0]
  start.y = g_src_coordinates[1]

  publisher_odom.publish(start)
  publisher_render.publish(Empty())

  #Load Map
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)
  x = np.arange(0,len(pixel_grid),40)
  y = np.arange(0,len(pixel_grid[0]),40)
  obstacle = False
  g_NUM_X_CELLS = 30
  g_NUM_Y_CELLS = 20
  g_WORLD_MAP = [0]*g_NUM_X_CELLS*g_NUM_Y_CELLS

  for i in x:
    for j in y:
      obstacle = False
      for k in range(i,i+40):
        for l in range(j,j+40):
          if pixel_grid[k][l] == 255:
            obstacle = True
      if obstacle == True:
        loc = ij_to_vertex_index(j/40,i/40)
        g_WORLD_MAP[loc] = 1

  #Find Path
  print("\nSource: (" + str(g_src_coordinates[0]) + ", " + str(1.2 - float(g_src_coordinates[1])) + ")" )
  print("Goal: (" + str(g_dest_coordinates[0]) + ", " + str(1.2 - float(g_dest_coordinates[1])) + ")" )
  src = ij_to_vertex_index(math.floor(16.6667*float(g_src_coordinates[0])), math.floor(16.6667*float(g_src_coordinates[1])))
  dest = ij_to_vertex_index(math.floor(16.6667*float(g_dest_coordinates[0])), math.floor(16.6667*float(g_dest_coordinates[1])))
  prev,coords = run_dijkstra(int(src))
  path = reconstruct_path(prev, int(src), int(dest))
  render_map(g_WORLD_MAP, path)
  if(path != []):
    if(len(path) > 1):
      for i in range(len(path)):
        if i != len(path) - 1:
          print(str(path[i]) + " -> "),
        else:
          print(str(path[i])),
    else:
      print(path[0])
  else:
    print("ERROR: NO POSSIBLE PATH")
  #Set the waypoints
  
  #Iterate over waypoints



    #IK helper functions and variables




# Lab 6 IK variables
int prev = NULL
int path = NULL
motor_message = Float32MultiArray()

def wheelRotation():
  moveLeft = value_array[1]
  moveForward = vaule_array[2]
  moveRight = value_array[3]

  if(moveForward < change_to_radians(1)):
    motor_message.data = [1.0, 1.0]
  elif(moveLeft < -change_to_radians(1)):
    motor_message.data = [-1.0, 1.0] #moving left wheel backwards to turn left
  elif(motorRight > change_to_radians(1)):
    motor_message.data = [1.0, -1.0] # moving right wheel backwards to turn right

  publisher_motor.publish(motor_message)
  msg = Empty()
  publisher_ping.publish(msg)

def change_to_radians(int deg):
   return deg * (3.14159 / 180)

float deg_error = 0.


def loop():
   long begin = millis()
   long end = 0
   int path_index = 0

   callback_update_odometry(data)

   switch(subscriber_state){
    case START:
      prev = run_dijkstra(world_map, ij_coordinates_to_xy_coordinates(0, 0))
      path = reconstruct_path(prev, ij_coordinates_to_xy_coordinates(0, 0), xy_coordinates_to_ij_coordinates(0, 0))
      if path[0] == ij_coordinates_to_xy_coordinates(0, 0):
        subsctiber_state = PATH_FOUND

    case PATH_FOUND:
      int i
      int j
      vertex_index_to_ij(path[path_index + 1], &i, &j)
      ij_coordinates_to_xy_coordinates(i, j, &pose2d_sparki_odometry.x, &pose2d_sparki_odometry.y)
      int m
      int n
      vertex_index_to_ij(path[path_index], &m, &n)
      if m < i:
        pose2d_sparki_odometry.theta = change_to_radians(0)
      if m > i:
        pose2d_sparki_odometry.theta = change_to_radians(180)
      if n < j:
        pose2d_sparki_odometry.theta = change_to_radians(90)
      if n > j:
        pose2d_sparki_odometry.theta = change_to_radians(-90)

      subscriber_state = NEXT_PART_POSE

      path_index++
      if path[path_index] == -1:
        subscriber_state = STOP

    case NEXT_PART_POSE:
    #finding if robot is at destination
      if pose2d_sparki_odometry.x != g_dest_coordinates.x && pose2d_sparki_odometry.y != g_dest_coordinates.y:
        subscriber_state = PATH_FOUND

   }

  

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  #part_1()
  #part_2(args)
  lab_6(args)