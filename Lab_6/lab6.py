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
g_MAP_SIZE_X = 1.8 # 2m wide
g_MAP_SIZE_Y = 1.2 # 1.5m tall
g_MAP_RESOLUTION_X = 0.06 # Each col represents 6cm
g_MAP_RESOLUTION_Y = 0.06 # Each row represents 6cm
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
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

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
  global g_NUM_Y_CELLS, g_NUM_X_CELLS
  global g_WORLD_MAP
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
          if u == 285:
            print("Val: ", val)
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
      """
      if not tmp[1] == (g_NUM_X_CELLS - 1) and not tmp[0] == (g_NUM_Y_CELLS - 1): 
        coord = ((tmp[0]*ratio + ratio/2)/666.66666,(tmp[1]*ratio + ratio/2)/666.66666)
        coords.append(coord)
      else:
        coord = ((tmp[0]*ratio - ratio/2)/666.66666,(tmp[1]*ratio - ratio/2)/666.66666)
        coords.append(coord)
      """
      tmp = ij_coordinates_to_xy_coordinates(tmp[0],tmp[1])
      coord = (tmp[0], 1.2 - tmp[1])
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

  # TODO: Insert your code here
  print(source_vertex)
  print(dest_vertex)
  final_path = []
  final_coords = []
  current = dest_vertex
  while current != source_vertex:
    if current == -1:
      return []
    else:
      final_path.insert(0,current)
      final_coords.insert(0, coords[current])
      #print(prev)
      #print(current)
      print("CURRENT: ", current)
      print("COORD: ", coords[current])
      current = prev[current]

  final_path.insert(0,source_vertex)
  # TODO: Insert your code here

  #print(final_path)
  return final_path, final_coords



def render_map(map_array, path_array):

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
        print(" x "),
      elif map_array[ij_to_vertex_index(j,i)] == 0:
        print(" . "),
      elif map_array[ij_to_vertex_index(j,i)] == 1:
        print("[ ]"),
      elif map_array[ij_to_vertex_index(j,i)] == 2:
        print(" S "),
      elif map_array[ij_to_vertex_index(j,i)] == 3:
        print(" D "),
    print('\n')




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


def updateDistance(pose_x, pose_y, dest_pose_x, dest_pose_y):
  return math.sqrt(pow(pose_x - dest_pose_x, 2)+ pow(pose_y - dest_pose_y, 2))

def correctTheta(raw):
  while(raw > math.pi):
    raw -= math.pi
  while(raw < 0):
    raw += math.pi
  return raw

def lab_6(args):
  global publisher_odom, pose2d_sparki_odometry
  global g_NUM_X_CELLS, g_NUM_Y_CELLS
  global g_WORLD_MAP
  publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
  publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
  init()
  g_src_coordinates = (args.src_coordinates[0], 1.2 - float(args.src_coordinates[1]))#we didn't need to subtract y from 1.2 but i realized 
  g_dest_coordinates = (args.dest_coordinates[0], 1.2 - float(args.dest_coordinates[1]))#that too late so theres some random subtractions
  
  start = Pose2D();
  start.x = float(g_src_coordinates[0])
  start.y = float(1.2 - g_src_coordinates[1])
  start.theta = 1.57 #trying to start the boi looking straight up to eliminate a bug i had but it didn't work
  end = Pose2D()
  end.x = float(g_dest_coordinates[0])
  end.y = float(args.dest_coordinates[1])

  print(start)
  publisher_odom.publish(start) #starting position, this works
  publisher_render.publish(Empty())


  start.y = float(1.2 - start.y)#fixing stuff with 1.2 subtraction math
  #Set the waypoints
  #Iterate over waypoints
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)#load image into array
  x = np.arange(0,len(pixel_grid),40) #idk why i did this sbut i did it and it works. bad memory management
  y = np.arange(0,len(pixel_grid[0]),40)

  #Detecting obstacles from image phase
  obstacle = False
  g_NUM_X_CELLS = 30
  g_NUM_Y_CELLS = 20
  g_WORLD_MAP = []
  #Generate a 20x30 array of 0's
  for i in range(g_NUM_Y_CELLS):
    for j in range(g_NUM_X_CELLS):
      g_WORLD_MAP.append(0)
  #loops through 40x40 sections of intensity matrix, if any part of the section has an obstacle, it is considered blocked
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
  print("\nSource: (" + str(g_src_coordinates[0]) + ", " + str(1.2 - float(g_src_coordinates[1])) + ")" )
  print("Goal: (" + str(g_dest_coordinates[0]) + ", " + str(1.2 - float(g_dest_coordinates[1])) + ")" )
  #converting from world to matrix coordinates for dijkstras
  tmpI, tmpJ = xy_coordinates_to_ij_coordinates(start.x, start.y)
  src = ij_to_vertex_index(tmpI, tmpJ)
  tmpI, tmpJ = xy_coordinates_to_ij_coordinates(float(g_dest_coordinates[0]), float(g_dest_coordinates[1]))
  dest = ij_to_vertex_index(tmpI, tmpJ)
  g_WORLD_MAP[src] = 0
  g_WORLD_MAP[dest] = 0
  #produce a path with dijkstras
  prev,coords = run_dijkstra(int(src))
  #returns the prev array and a corresponding coordinate array
  prev_path, coords_path = reconstruct_path(prev, coords, src, dest)
  render_map(g_WORLD_MAP, prev_path)
  #prints out the path
  if(prev_path != []):
    if(len(prev_path) > 1):
      for i in range(len(prev_path)):
        if i != len(prev_path) - 1:
          print(str(prev_path[i]) + " -> "),
        else:
          print(str(prev_path[i])),
    else:
      print(prev_path[0])
  else:
    print("ERROR: NO POSSIBLE PATH")
  print('\n')

  #sectioned movement phase
  rate = rospy.Rate(1/g_CYCLE_TIME)#generates the wait time
  motor_message = Float32MultiArray()#message to be published
  publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)#inititalizes the render publisher
  print(coords_path)
  tmp = xy_coordinates_to_ij_coordinates(.9,.3)
  tmp = ij_to_vertex_index(tmp[0],tmp[1])
  tmp = vertex_index_to_ij(tmp)
  tmp = ij_coordinates_to_xy_coordinates(tmp[0],tmp[1])
  print(tmp)

  for i in range(1, len(coords_path)):#loop through every coordinate 
    coord = coords_path[i]
    distance = updateDistance(pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, coord[0], coord[1])
    while  distance > .02: #if sparki isn't within 5mm of target, keep moving
      distance = updateDistance(pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, coord[0], coord[1])
      goal_theta = math.atan2(coord[1] - pose2d_sparki_odometry.y, coord[0] - pose2d_sparki_odometry.x)
      goal_theta = correctTheta(goal_theta)
      print("GOAL: ", coord)
      print("ACTUAL: ", pose2d_sparki_odometry)
      while abs(goal_theta - pose2d_sparki_odometry.theta) > (math.pi/100): #attempting to align sparki in the proper direction
        #print("GOAL: ", goal_theta)
        #print("ACTUAL: ", pose2d_sparki_odometry.theta)
        if pose2d_sparki_odometry.theta >= 6.28:
          tmp = Pose2D()
          tmp.x = pose2d_sparki_odometry.x
          tmp.y = pose2d_sparki_odometry.y
          tmp.theta = 0
          publisher_odom.publish(tmp)
        motor_message.data = [1.0, -1.0]

        publisher_motor.publish(motor_message)
        rate.sleep()
        motor_message.data = [0,0]
        publisher_motor.publish(motor_message)
        publisher_render.publish(Empty())
      #Sparki should be lined up at this point, move forward until close enough to go to next coordinate
      motor_message.data = [1.0, 1.0]
      publisher_motor.publish(motor_message)
      rate.sleep()
      motor_message.data = [0,0]
      publisher_motor.publish(motor_message)
      publisher_render.publish(Empty())

  print("\nYou have arrived at your destination")
  

#CURRENT BUGS
#If sparki pivots right, his odometry goes negative and will never reach the target
#Homie just pivots in circles at times
#Coordinate array may not be including the proper values/might be backwards


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  #part_1()
  #part_2(args)
  lab_6(args)