from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import numpy as np


WALL_OFFSET = 2.
#CYLINDER_POSITION = np.array([.3, .2], dtype=np.float32)

CYLINDER_POSITION = np.array([.0, .5], dtype=np.float32)
CYLINDER_POSITION2 = np.array([.5, .0], dtype=np.float32)

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
  #to calculate the magnitude
  #(x2-x1)^2 + (y2-y1)^2
  mag=np.sqrt((x2-x1)**2+(y2-y1)**2)
  #get the vector's direction and normalise it
  direction=normalize([x2-x1, y2-y1])
 
  #multiply the direction by the calucated magnitude and and random variation
  #to avoid zero vector sum from perpendicular vectors
  direction[0]*=mag+(np.random.uniform(-2,2))
  direction[1]*=mag+(np.random.uniform(-2,2))

  #multiply the direction by the calucated magnitude
  #direction[0]*=mag
  #direction[1]*=mag
  
  #OPTIONAL: get the max vector's magnitude
  magnitude=np.linalg.norm(direction)
  if(magnitude>4.8):
    print('magnitude:', magnitude)
  
  return cap(direction, MAX_SPEED)
  #direction #cap(directionv,MAX_SPEED)


def get_velocity_to_avoid_obstacles(position, obstacle_positions, obstacle_radii):
  v = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to avoid the obstacles
  # In the worst case there might a large force pushing towards the
  # obstacles (consider what is the largest force resulting from the
  # get_velocity_to_reach_goal function). Make sure to not create
  # speeds that are larger than max_speed for each obstacle. Both obstacle_positions
  # and obstacle_radii are lists.

  #a funciton that calculates the force radiating of an obstacle
  def get_force(x,y,obstacle_pos,rad):
    obs_x=obstacle_pos[0]
    obs_y=obstacle_pos[1]

    #magnitude is calculated by dividing the obstacle area by the distance to it
    #the drawbakc is that it only works with circular obstacles
    magnitude=(3.14*rad**2)/np.sqrt((obs_x-x)**2+(obs_y-y)**2)

    #calculate the vector direction and normalise
    direction=normalize([obs_x-x, obs_y-y])

    #multiply the directon by the magnitude
    direction[0]*=magnitude
    direction[1]*=magnitude

    return direction

    
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

     #get the radius
      radius=obstacle_radii[iterator]
    #calc the obstacle force 
      obstacle_force=get_force(x1,y1, obstacle,radius)
    #assign the force to the vector
      v[0]+=obstacle_force[0]
      v[1]+=obstacle_force[1]
            
      iterator+=1


##optional, calc the magnitude
  magnit=np.linalg.norm(v)
      
#sinc objects need to repel the robot, multiply by -1 to inverse the vector
  v=v*(-1)

  return v#cap(v, -0.25)


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
      #[CYLINDER_POSITION,CYLINDER_POSITION2],
      #[CYLINDER_RADIUS,CYLINDER_RADIUS])
       [CYLINDER_POSITION ],
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
 # ax.add_artist(plt.Circle(CYLINDER_POSITION, CYLINDER_RADIUS, color='gray'))


  ax.add_artist(plt.Circle([0.5,0], CYLINDER_RADIUS, color='lightgray'))
  ax.add_artist(plt.Circle([0,0.5], CYLINDER_RADIUS, color='gray'))

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
