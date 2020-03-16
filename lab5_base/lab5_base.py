import copy
import math
import random
import argparse
from PIL import Image
import numpy as np
from pprint import pprint

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

  # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
  

  # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!

  """
  map_size = g_NUM_X_CELLS * g_NUM_Y_CELLS
  source_x, source_y = vertex_index_to_ij(source_vertex)

  world_map = g_WORLD_MAP
  for i in range(map_size):
    for j in range(map_size):
      #curr_x, curr_y = vertex_index_to_ij(j)
      val = dist[j] + get_travel_cost(i,j)
      print(val)
      #print(val)
      if val < dist[j]:
        Q_cost = val
        dist[j] = val
        prev[j] = i
  """

  """
  for i in range(map_size):
    dist[i] = float('inf')
    prev[i] = -1
    if(not world_map[i] == 1):
      Q_cost.append((i, -1))
  dist[source_vertex] = 0
  while(len(Q_cost) > 0):
    distance = 1000
    index = -1
    #Find shortest in Q
    for i in range(len(Q_cost)):
      Q_current = Q_cost[i]
      if((not dist[Q_current[0]] ==-1) and dist[Q_current[0]] < distance):
        index = i
        Q_cost[i] = (Q_cost[i][0], dist[Q_cost[i][0]])
        distance = Q_cost[i][1]
    u = Q_cost.pop(index)
    u_x, u_y = vertex_index_to_ij(u[0])
    #Top neighbor
    v = ij_to_vertex_index(u_x, u_y-1)
    adjacent = get_travel_cost(u[0], v)
    if(adjacent != 1000):
      alt = u[1] + adjacent
      if(v > 0 and v < map_size):
        if(dist[v] == -1 or alt < dist[v]):
          dist[v] = alt
          prev[v] = u[0]
    #Bottom neighbor
    v = ij_to_vertex_index(u_x, u_y+1)
    adjacent = get_travel_cost(u[0], v)
    if(adjacent != 1000):
      alt = u[1] + adjacent
      if(v > 0 and v < map_size):
        if(dist[v] == -1 or alt < dist[v]):
          dist[v] = alt
          prev[v] = u[0]
    #Right neighbor
    v = ij_to_vertex_index(u_x+1, u_y)
    adjacent = get_travel_cost(u[0], v)
    if(adjacent != 1000):
      alt = u[1] + adjacent
      if(v > 0 and v < map_size):
        if(dist[v] == -1 or alt < dist[v]):
          dist[v] = alt
          prev[v] = u[0]
    #Left neigbhor 
    v = ij_to_vertex_index(u_x-1, u_y)
    adjacent = get_travel_cost(u[0], v)
    if(adjacent != 1000):
      alt = u[1] + adjacent
      if(v > 0 and v < map_size):
        if(dist[v] == -1 or alt < dist[v]):
          dist[v] = alt
          prev[v] = u[0]
  # Return results of algorithm run
  """
  return prev


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

  global g_NUM_X_CELLS
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
      elif g_WORLD_MAP[ij_to_vertex_index(j,i)] == 0:
        print(" . "),
      elif g_WORLD_MAP[ij_to_vertex_index(j,i)] == 1:
        print("[ ]"),
    print('\n')


def part_1():
  global g_WORLD_MAP

  global g_dest_coordinates
  global g_src_coordinates

  # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles

  g_WORLD_MAP = [0, 0, 0, 0, 1, 1, 0, 0, 0]
  g_NUM_X_CELLS = 3
  g_NUM_Y_CELLS = 3

  # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path
  print("Source: (" + str(g_src_coordinates[0]) + ", " + str(g_src_coordinates[1]) + ")" )
  print("Goal: (" + str(g_dest_coordinates[0]) + ", " + str(g_dest_coordinates[1]) + ")" )

  source_int = ij_to_vertex_index(g_src_coordinates[0], g_src_coordinates[1])
  path = reconstruct_path(run_dijkstra(source_int), ij_to_vertex_index(g_src_coordinates[0], g_src_coordinates[1]), 8)
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

  g_src_coordinates = (args.src_coordinates[0], args.src_coordinates[1])
  g_dest_coordinates = (args.dest_coordinates[0], args.dest_coordinates[1])

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
        loc = ij_to_vertex_index(j/40,i/40)
        g_WORLD_MAP[loc] = 1

  print("Source: (" + str(g_src_coordinates[0]) + ", " + str(g_src_coordinates[1]) + ")" )
  print("Goal: (" + str(g_dest_coordinates[0]) + ", " + str(g_dest_coordinates[1]) + ")" )
  src = ij_to_vertex_index(math.floor(16.6667*g_src_coordinates[0]), math.floor(16.6667*g_src_coordinates[1]))
  dest = ij_to_vertex_index(math.floor(16.6667*g_dest_coordinates[0]), math.floor(16.6667*g_dest_coordinates[1]))
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
  #print(g_WORLD_MAP)

  '''
  TODO -
  1) Compute the g_WORLD_MAP -- depending on the resolution, you need to decide if your cell is an obstacle cell or a free cell.
  2) Run Dijkstra's to get the plan
  3) Show your plan/path on the image
  Feel free to add more helper functions
  '''

  #### Your code goes here ####




if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()


  #part_1()
  part_2(args)