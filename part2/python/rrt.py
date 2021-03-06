from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import re
import scipy.signal
import yaml


# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.105 / 2.
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)  # Any orientation is good.
START_POSE = np.array([-1.5, -1.5, 0.], dtype=np.float32)
MAX_ITERATIONS = 500


def sample_random_position(occupancy_grid):
  pos = np.zeros(2, dtype=np.float32)
 
  # MISSING: Sample a valid random position (do not sample the yaw).
  # The corresponding cell must be free in the occupancy grid.

  #occupancy grid size is 147456
  #meaning one side is 384 pixels

 # position=np.random.uniform(-2.5,2.5,2)
  DEBUG=False
  
  is_occupied=True
  is_free=False

  #enter while loop which makes sure that the generated positions are valid
  while(not is_free):

    #generate random positions that fit within the arena
      pos=np.random.uniform(-2,2,2)

      if(DEBUG):
        print('grid pos', pos)

    #check the occupancy grid functions
      is_free=occupancy_grid.is_free(pos) 
      is_occupied=occupancy_grid.is_occupied(pos) 

      if (is_free):
        break # 

      if(DEBUG):
        print('not free try again')

  if(DEBUG):
     print('success, it was free')

  
  return pos


def adjust_pose(node, final_position, occupancy_grid):
  final_pose = node.pose.copy()
  final_pose[:2] = final_position
  final_node = Node(final_pose)

  curr_pose=node.pose[:2]
  
  DEBUG=False

#######################################################
################## HELPER FUNCTIONS ###################
######################################################

#finds midpoint coordinates between two nodes
  def find_midpoint(node_a, node_b):
    midpoint=(node_a[X]+node_b[X])/2,(node_a[Y]+node_b[Y])/2
    return midpoint

#finds the line's gradient
#if y=mx+c, this function returns the gradient m
  def find_gradient(pos1, pos2):
    x1=pos1[X]
    y1=pos1[Y]

    x2=pos2[X]
    y2=pos2[Y]

    #edge case one, no line present
    if(x1==x2):
        #print('infinite gradient, error')
        return np.inf
    else:
        #print('finding gradient', x1,x2,y1,y2)
        return (y2-y1)/(x2-x1)

#finds a perpendicular line
  def find_perpendicular(gradient):
    #edge case one, no gradient just a constant
    if(gradient==0):
        #print('infinite gradient, error')
        return np.inf
     #edge case one, gradient is infinite, set gradient to constant
    elif gradient==np.inf:
        #print('infinite gradient, error')
        return 0
    else:#the usual
        return -1/gradient

#fins the intersect positions between two points
  def find_intersect(m1,m2,c1,c2):

    #edge case one, constant1 is  infinite, set it to 0 aka no constant
    if c1[0] ==np.inf:
        x=c1[1]#or just 0
        return [x, m2*x+c2[0]]
     #edge case two, constant2 is infinite, set it to 0 aka no constant
    elif c2[0]==np.inf:
        x=c2[1] #or just 0
        return [x, m1*x+c1[0]]
    else:#the usual
        #print('intersect', [m1,m2,c1,c2])
        x=(c2[0]-c1[0])/(m1-m2)
        y=m1*x+c1[0]

    return [x,y]

#finds the constant of a straight line
#if y=mx+c
#returns an array of values to deal with edge cases
  def find_c(node, gradient):
    #edge case one, the gradient is infinite, the array has node's X position
    if (gradient==np.inf):
        return [np.inf, node[X]]
    else:#the usual
        y=node[Y]
        x=node[X]
        m=gradient
    return [y-m*x,0]


##converts cartesian to polar
  def cart2pol(x, y):
      rho = np.sqrt(x**2 + y**2)
      phi = np.arctan2(y, x)
      return(rho, phi)

  #converts polar to cartesian
  def pol2cart(rho, phi):
      x = rho * np.cos(phi)
      y = rho * np.sin(phi)
      return(x, y)

#get perpendicualr vector for another vector
  def perpendicular(a):
    b=np.empty_like(a)
    b[0]=-a[1]
    b[1]=a[0]
    return b
 ############################################################
#######################################################

#in steps 1-7 we try to get the coordinates of the circle that has start and finish 
#nodes on it's circumference.

#step 1 get the midpoint between the start S and finish F nodes
  midpoint=find_midpoint(node.position, final_position)

#step 2 get the gradient of the line connecting nodes S and F
  gradient_line_1=find_gradient(node.position, final_position)

#step 3 find a line that perpendicular to the line connecting S and F
  gradient_line_2=find_perpendicular(gradient_line_1)

#step 4 find the constant of the perpendicular line to S and F at the midpoint
  c2=find_c(midpoint,gradient_line_2)

#get the S node's yaw
  node_gradient=np.tan(node.yaw)

#step 5 find a line that#s perpendicular to S's yaw
  gradient_line_3=find_perpendicular(node_gradient)

#step 6 find the constant of the perpendicular line to the node's S yaw
  c3=find_c(node.position,gradient_line_3)

#step 7 -now that we check where the line passing through the midpoint (perpendicularly)
#intersects with the line that's perpendiculat to S node's yaw.
#this will give the center coordinates of the circle C that's passing throught 
#S and F
  center_circle=find_intersect(gradient_line_2, gradient_line_3, c2,c3)

#in steps  8-10 we get the F node's yaw

#step8, get the gradient of the line C-F
  gradient_line_4=find_gradient(center_circle, final_position)
#step 9, get the gradient of the line perpendicular to CF
  gradient_final=find_perpendicular(gradient_line_4)


##the direction of the perpendicular vector to center circle and final pos
  du=node.position-center_circle

#step 10, get the cross product of the finish and start vectors
  if np.cross(node.direction, du).item()>0: 
  
       if  (final_position[0]-center_circle[0])>=0 and (final_position[1]-center_circle[1])>=0:
            final_node.pose[YAW]= -np.arctan(np.absolute(gradient_final))

       elif(final_position[0]-center_circle[0])>=0 and (final_position[1]-center_circle[1])<0:
            final_node.pose[YAW]=np.pi+np.arctan(gradient_final)

       elif(final_position[0]-center_circle[0])<0  and (final_position[1]-center_circle[1])<0:
            final_node.pose[YAW]=np.pi - np.arctan(np.absolute(gradient_final))

       elif(final_position[0]-center_circle[0])<0  and (final_position[1]-center_circle[1])>=0:
            final_node.pose[YAW]=np.arctan(gradient_final)

       else:
            return None
            print("NaN")

  else:
        if  (final_position[0]-center_circle[0])>=0 and (final_position[1]-center_circle[1])>=0:
            final_node.pose[YAW] = np.pi - np. arctan(np.absolute(gradient_final))

        elif(final_position[0]-center_circle[0])>=0 and (final_position[1]-center_circle[1])<0:
            final_node.pose[YAW]= np.arctan(gradient_final)

        elif(final_position[0]-center_circle[0])<0  and (final_position[1]-center_circle[1])<0:
            final_node.pose[YAW]=-np.arctan(np.absolute(gradient_final))

        elif(final_position[0]-center_circle[0])<0  and (final_position[1]-center_circle[1])>=0:
            final_node.pose[YAW]=np.pi+np.arctan(gradient_final)

        else:
            return None
            print("NaN")

#update the pose
  final_node.pose[X]=final_position[0]
  final_node.pose[Y]=final_position[1]

#angle C-S
  theta1=np.arctan2(du[1], du[0])
  dv=final_position-center_circle

#angle C-F
  theta2=np.arctan2(dv[1], dv[0])
  
#angle S-C-F
  theta_delta=np.absolute(theta1-theta2)

  ###########################################################


 ##get the fitted circle from the find_circle
  new_node=find_circle(node, final_node)[0]
  node_rad=find_circle(node, final_node)

  ##get the circles center positoin and radius
  rad_pos=node_rad[0]
  rad_len=node_rad[1]
  
  #get the center offset to transform the circle to 0.0 so we can use polar coords
  x_offset=center_circle[X]
  y_offset=center_circle[Y]

  #transform the two given node points to 0,0
  cart_coords_A=cart2pol(node.pose[X]-x_offset,node.pose[Y]-y_offset)
  cart_coords_B=cart2pol(final_position[X]-x_offset,final_position[Y]-y_offset)

  #get the polar angles for the two given points
  angle_A=cart_coords_A[1]
  angle_B=cart_coords_B[1]

  ##interpolate 20 steps inbetween the two angles
  interp=np.linspace(angle_A, angle_B, 20)

  #check the dot positions using the interpolated angles
  #here we use polar coordinates so it's easier to calculate i
  for angle in interp:
  ##transform polar to cartesian + remove the offset so  so we check it's real location
    cart_xy=pol2cart(rad_len, angle)+rad_pos
    #check if the real location is frees
    available=occupancy_grid.is_free(cart_xy)
    #print(angle, cart_xy, available)
    if(not available):
        return None
  
 
  # MISSING: Check whether there exists a simple path that links node.pose
  # to final_position. This function needs to return a new node that has
  # the same position as final_position and a valid yaw. The yaw is such that
  # there exists an arc of a circle that passes through node.pose and the
  # adjusted final pose. If no such arc exists (e.g., collision) return None.
  # Assume that the robot always goes forward.
  # Feel free to use the find_circle() function below.

  #final_node.pose[YAW]=0
  return final_node


# Defines an occupancy grid.
class OccupancyGrid(object):
  def __init__(self, values, origin, resolution):
    self._original_values = values.copy()
    self._values = values.copy()
    # Inflate obstacles (using a convolution).
    inflated_grid = np.zeros_like(values)
    inflated_grid[values == OCCUPIED] = 1.
    w = 2 * int(ROBOT_RADIUS / resolution) + 1
    inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
    self._values[inflated_grid > 0.] = OCCUPIED
    self._origin = np.array(origin[:2], dtype=np.float32)
    self._origin -= resolution / 2.
    assert origin[YAW] == 0.
    self._resolution = resolution

  @property
  def values(self):
    return self._values

  @property
  def resolution(self):
    return self._resolution

  @property
  def origin(self):
    return self._origin

  def draw(self):
    plt.imshow(self._original_values.T, interpolation='none', origin='lower',
               extent=[self._origin[X],
                       self._origin[X] + self._values.shape[0] * self._resolution,
                       self._origin[Y],
                       self._origin[Y] + self._values.shape[1] * self._resolution])
    plt.set_cmap('gray_r')

  def get_index(self, position):
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 2:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      return (idx[:, 0], idx[:, 1])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    return tuple(idx)

  def get_position(self, i, j):
    return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

  def is_occupied(self, position):
    return self._values[self.get_index(position)] == OCCUPIED

  def is_free(self, position):
    return self._values[self.get_index(position)] == FREE


# Defines a node of the graph.
class Node(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node):
    self._neighbors.append(node)

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose[:2]

  @property
  def yaw(self):
    return self._pose[YAW]
  
  @property
  def direction(self):
    return np.array([np.cos(self._pose[YAW]), np.sin(self._pose[YAW])], dtype=np.float32)

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c


def rrt(start_pose, goal_position, occupancy_grid):
  # RRT builds a graph one node at a time.
  graph = []
  start_node = Node(start_pose)
  final_node = None
  if not occupancy_grid.is_free(goal_position):
    print('Goal position is not in the free space.')
    return start_node, final_node
  graph.append(start_node)
  for _ in range(MAX_ITERATIONS): 
    position = sample_random_position(occupancy_grid)
    # With a random chance, draw the goal position.
    if np.random.rand() < .05:
      position = goal_position
    # Find closest node in graph.
    # In practice, one uses an efficient spatial structure (e.g., quadtree).
    potential_parent = sorted(((n, np.linalg.norm(position - n.position)) for n in graph), key=lambda x: x[1])
    # Pick a node at least some distance away but not too far.
    # We also verify that the angles are aligned (within pi / 4).
    u = None
    for n, d in potential_parent:
      if d > .2 and d < 1.5 and n.direction.dot(position - n.position) / d > 0.70710678118:
        u = n
        break
    else:
      continue
    v = adjust_pose(u, position, occupancy_grid)
    if v is None:
      continue
    u.add_neighbor(v)
    v.parent = u
    graph.append(v)
    if np.linalg.norm(v.position - goal_position) < .2:
      final_node = v
      break
  return start_node, final_node


def find_circle(node_a, node_b):
  def perpendicular(v):
    w = np.empty_like(v)
    w[X] = -v[Y]
    w[Y] = v[X]
    return w
  db = perpendicular(node_b.direction)
  dp = node_a.position - node_b.position
  t = np.dot(node_a.direction, db)
  if np.abs(t) < 1e-3:
    # By construction node_a and node_b should be far enough apart,
    # so they must be on opposite end of the circle.
    center = (node_b.position + node_a.position) / 2.
    radius = np.linalg.norm(center - node_b.position)
  else:
    radius = np.dot(node_a.direction, dp) / t
    center = radius * db + node_b.position
  return center, np.abs(radius)


def read_pgm(filename, byteorder='>'):
  """Read PGM file."""
  with open(filename, 'rb') as fp:
    buf = fp.read()
  try:
    header, width, height, maxval = re.search(
        b'(^P5\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n]\s)*)', buf).groups()
  except AttributeError:
    raise ValueError('Invalid PGM file: "{}"'.format(filename))
  maxval = int(maxval)
  height = int(height)
  width = int(width)
  img = np.frombuffer(buf,
                      dtype='u1' if maxval < 256 else byteorder + 'u2',
                      count=width * height,
                      offset=len(header)).reshape((height, width))
  return img.astype(np.float32) / 255.


def draw_solution(start_node, final_node=None):
  ax = plt.gca()

  def draw_path(u, v, arrow_length=.1, color=(.8, .8, .8), lw=1):
    du = u.direction
    plt.arrow(u.pose[X], u.pose[Y], du[0] * arrow_length, du[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    dv = v.direction
    plt.arrow(v.pose[X], v.pose[Y], dv[0] * arrow_length, dv[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    center, radius = find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    if np.cross(u.direction, du).item() > 0.:
      theta1, theta2 = theta2, theta1
    ax.add_patch(patches.Arc(center, radius * 2., radius * 2.,
                             theta1=theta1 / np.pi * 180., theta2=theta2 / np.pi * 180.,
                             color=color, lw=lw))

  points = []
  s = [(start_node, None)]  # (node, parent).
  while s:
    v, u = s.pop()
    if hasattr(v, 'visited'):
      continue
    v.visited = True
    # Draw path from u to v.
    if u is not None:
      draw_path(u, v)
    points.append(v.pose[:2])
    for w in v.neighbors:
      s.append((w, v))

  points = np.array(points)
  plt.scatter(points[:, 0], points[:, 1], s=10, marker='o', color=(.8, .8, .8))
  if final_node is not None:
    plt.scatter(final_node.position[0], final_node.position[1], s=10, marker='o', color='k')
    # Draw final path.
    v = final_node
    while v.parent is not None:
      draw_path(v.parent, v, color='k', lw=2)
      v = v.parent


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Uses RRT to reach the goal.')
  parser.add_argument('--map', action='store', default='map', help='Which map to use.')
  args, unknown = parser.parse_known_args()

  # Load map.
  with open(args.map + '.yaml') as fp:
    data = yaml.load(fp)
  img = read_pgm(os.path.join(os.path.dirname(args.map), data['image']))
  occupancy_grid = np.empty_like(img, dtype=np.int8)
  occupancy_grid[:] = UNKNOWN
  occupancy_grid[img < .1] = OCCUPIED
  occupancy_grid[img > .9] = FREE
  # Transpose (undo ROS processing).
  occupancy_grid = occupancy_grid.T
  # Invert Y-axis.
  occupancy_grid = occupancy_grid[:, ::-1]
  occupancy_grid = OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

  # Run RRT.
  start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)

  # Plot environment.
  fig, ax = plt.subplots()
  occupancy_grid.draw()
  plt.scatter(.3, .2, s=10, marker='o', color='green', zorder=1000)
  draw_solution(start_node, final_node)
  plt.scatter(START_POSE[0], START_POSE[1], s=10, marker='o', color='green', zorder=1000)
  plt.scatter(GOAL_POSITION[0], GOAL_POSITION[1], s=10, marker='o', color='red', zorder=1000)
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - 2., 2. + .5])
  plt.ylim([-.5 - 2., 2. + .5])
  plt.show()
  
