from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import numpy as np


WALL_OFFSET = 2.
CYLINDER_POSITION = np.array([.3, .2], dtype=np.float32)
CYLINDER_RADIUS = .3
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)
START_POSITION = np.array([-1.5, -1.5], dtype=np.float32)
MAX_SPEED = .5


def get_velocity_to_reach_goal(position, goal_position):
  v = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to reach goal_position
  # assuming that there are no obstacles.
 
  #get current and goal positions
  current=position

  #the goal position is always 1.5 and 1.5
  goal=goal_position

  ## get the x values for current and goal positions
  x1=current[0]
  x2=goal[0]

  ##get the y value sfor current and goal positions
  y1=current[1]
  y2=goal[1]
  
  #get the distance between current position and the goal
  #(x2-x1)^2 + (y2-y1)^2
  dist=np.sqrt((x2-x1)**2+(y2-y1)**2)

  #subtract the current vector field values from the distance
  v[0]=dist-x1
  v[1]=dist-y1

  #OPTIONAL: get the max vector's magnitude
  magnitude=np.linalg.norm(v)
  #if(magnitude>9):
  #  print('magnitude:', magnitude)
  
  return v


def get_velocity_to_avoid_obstacles(position, obstacle_positions, obstacle_radii):
  v = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to avoid the obstacles
  # In the worst case there might a large force pushing towards the
  # obstacles (consider what is the largest force resulting from the
  # get_velocity_to_reach_goal function). Make sure to not create
  # speeds that are larger than max_speed for each obstacle. Both obstacle_positions
  # and obstacle_radii are lists.

 #get current and goal positions
  current=position

  #the obstacle position [single array entry?]
  ##for now only assume there is one obstacle
  obstacles=obstacle_positions

  ## get the x values for current and obstacle positions
  x1=current[0]

  ##get the y value sfor current and obstacle positions
  y1=current[1]

  dists_to_obstacles=[]

  iterator=0
  for obstacle in obstacles:
      #get the distance between current position and the obstacle
      #(x2-x1)^2 + (y2-y1)^2
      x2_obs=obstacle[0]
      y2_obs=obstacle[1]

      dist_to_obst=np.sqrt((x2_obs-x1)**2+(y2_obs-y1)**2)-obstacle_radii[iterator]
      dists_to_obstacles.append(dist_to_obst)
      iterator+=1
        
# 
  # ##get the distance from the current position and the obstacle
  # print('obs_positions',obstacle_positions)
  # print('obs_radii',obstacle_radii)

  #obstacle radii multiplier
  #iterator2=0
  for dists in dists_to_obstacles:
      v[0]=((dists-x1)*-1)
      v[1]=((dists-y1)*-1)

      magnitude=np.linalg.norm(v)

      #print(v,magnitude)
  print(v,cap(v,MAX_SPEED))

      ##make sure v isn't more than max speed
  return cap(v, 0.5)


def normalize(v):
  n = np.linalg.norm(v)
  if n < 1e-2:
    return np.zeros_like(v)
  return v / n


def cap(v, max_speed):
  n = np.linalg.norm(v)
  if n > max_speed:
    return v / n * max_speed
  return v


def get_velocity(position, mode='all'):
  if mode in ('goal', 'all'):
    v_goal = get_velocity_to_reach_goal(position, GOAL_POSITION)
  else:
    v_goal = np.zeros(2, dtype=np.float32)
  if mode in ('obstacle', 'all'):
    v_avoid = get_velocity_to_avoid_obstacles(
      position,
      [CYLINDER_POSITION],
      [CYLINDER_RADIUS])
  else:
    v_avoid = np.zeros(2, dtype=np.float32)
  v = v_goal + v_avoid
  return cap(v, max_speed=MAX_SPEED)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs obstacle avoidance with a potential field')
  parser.add_argument('--mode', action='store', default='all', help='Which velocity field to plot.', choices=['obstacle', 'goal', 'all'])
  args, unknown = parser.parse_known_args()

  fig, ax = plt.subplots()
  # Plot field.
  X, Y = np.meshgrid(np.linspace(-WALL_OFFSET, WALL_OFFSET, 30),
                     np.linspace(-WALL_OFFSET, WALL_OFFSET, 30))
  U = np.zeros_like(X)
  V = np.zeros_like(X)
  for i in range(len(X)):
    for j in range(len(X[0])):
      velocity = get_velocity(np.array([X[i, j], Y[i, j]]), args.mode)
      U[i, j] = velocity[0]
      V[i, j] = velocity[1]
  plt.quiver(X, Y, U, V, units='width')

  # Plot environment.
  ax.add_artist(plt.Circle(CYLINDER_POSITION, CYLINDER_RADIUS, color='gray'))
  plt.plot([-WALL_OFFSET, WALL_OFFSET], [-WALL_OFFSET, -WALL_OFFSET], 'k')
  plt.plot([-WALL_OFFSET, WALL_OFFSET], [WALL_OFFSET, WALL_OFFSET], 'k')
  plt.plot([-WALL_OFFSET, -WALL_OFFSET], [-WALL_OFFSET, WALL_OFFSET], 'k')
  plt.plot([WALL_OFFSET, WALL_OFFSET], [-WALL_OFFSET, WALL_OFFSET], 'k')

  # Plot a simple trajectory from the start position.
  # Uses Euler integration.
  dt = 0.01
  x = START_POSITION
  positions = [x]
  for t in np.arange(0., 20., dt):
    v = get_velocity(x, args.mode)
    x = x + v * dt
    positions.append(x)
  positions = np.array(positions)
  plt.plot(positions[:, 0], positions[:, 1], lw=2, c='r')

  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - WALL_OFFSET, WALL_OFFSET + .5])
  plt.ylim([-.5 - WALL_OFFSET, WALL_OFFSET + .5])
  plt.show()
