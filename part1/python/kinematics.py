from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import collections
import matplotlib
import matplotlib.pylab as plt
import numpy as np
from numpy import random
import time

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Drawing constants.
REFRESH_RATE = 1. / 15.


def euler(current_pose, t, dt):
  next_pose = current_pose.copy()
  u = 0.25
  w = np.cos(t)
  w = np.cos(np.floor(t))

  #forward velocity
   # x=u*cos(theta)# where theta is time
   # y=u*sin(theta)# where theta is time

  #rotational velocity
 
  print(t,current_pose, '\n')

    #http://geofhagopian.net/m2c/M2C-S18/euler_method.pdf
  fx=lambda curr_po:(u*np.cos(current_pose[YAW])) # curr_po is the current_pose
  fy=lambda curr_po:(u*np.sin(current_pose[YAW])) # curr_po is the current pose

  print(current_pose[YAW],w)

  next_pose[X]=current_pose[X] + dt*fx(current_pose[X])
  next_pose[Y]=current_pose[Y] + dt*fy(current_pose[Y]) 

  next_pose[YAW]=current_pose[YAW] + dt*w

  # MISSING: Use EulerEuler's integration method to return the next pose of our robot.
  # https://en.wikipedia.org/wiki/Euler_method
  # t is the current time.
  # dt is the time-step duration.
  # current_pose[X] is the current x position.
  # current_pose[Y] is the current y position.
  # current_pose[YAW] is the current orientation of the robot.
  # Update next_pose[X], next_pose[Y], next_pose[YAW].

  return next_pose


def rk4(current_pose, t, dt):
  next_pose = current_pose.copy()
  #next_pose+=0.1
  print('next_pose', next_pose)
  # MISSING: Use classical Runge-Kutta to return the next pose of our robot.
  # https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

##NOT SURE IF I NEED THESE
  u = 0.25
  w = np.cos(t)
  w = np.cos(np.floor(t))
 
  fx=lambda t,curr_po:(u*np.cos(w)) # curr_po is the current_pose
  fy=lambda t,curr_po:(u*np.sin(w)) # curr_po is the current pose

###ATTENTION PAN'S CODE BELOW, TO BE MODIFIED!!!###
  print('current_pose', current_pose[X],dt)
  xk1=u*np.cos(current_pose[YAW])
  xk3=u*np.cos(current_pose[YAW])
  xk2=u*np.cos(current_pose[YAW])
  xk4=u*np.cos(current_pose[YAW])

  yk1=u*np.sin(current_pose[YAW])
  yk3=u*np.sin(current_pose[YAW])
  yk2=u*np.sin(current_pose[YAW])
  yk4=u*np.sin(current_pose[YAW])

  thetak1=np.cos(np.floor(t        ))
  thetak2=np.cos(np.floor(t + dt/2.))
  thetak3=np.cos(np.floor(t + dt/2.))
  thetak4=np.cos(np.floor(t + dt   ))

  next_pose[YAW]=current_pose[YAW]+dt*(thetak1+2.*thetak2+2.*thetak3+thetak4)/6.
  next_pose[X]=current_pose[X] + dt*(xk1 + 2.*xk2 + 2.*xk3 + xk4)/6.
  next_pose[Y]=current_pose[Y] + dt*(yk1 + 2.*yk2 + 2.*yk3 + yk4)/6.

  # def compute_ks(y_val, modifier):
# 
  # #based off of https://mathworld.wolfram.com/Runge-KuttaMethod.html
    # if(modifier == 'y'):
        # k1 = dt * fy(t         , y_val       )
        # k2 = dt * fy(t + 0.5*dt, y_val+0.5*k1)
        # k3 = dt * fy(t + 0.5*dt, y_val+0.5*k2)
        # k4 = dt * fy(t +     dt, y_val+    k3)
    # else:#if computing x coord
        # k1 = dt * fx(t         , y_val       )
        # k2 = dt * fx(t + 0.5*dt, y_val+0.5*k1)
        # k3 = dt * fx(t + 0.5*dt, y_val+0.5*k2)
        # k4 = dt * fx(t +     dt, y_val+    k3)
# 
    # computed_k=y_val+(1.0/6.0)*(k1+2*k2+2*k3+k4)
# 
    # return computed_k
 # 

  #next_pose[X]=compute_ks(current_pose[X],'x')
  #next_pose[Y]=compute_ks(current_pose[Y],'y')  
  #next_pose[YAW]=current_pose[YAW] + dt*w


  return next_pose


def main(args):
  print('Using method {}'.format(args.method))
  integration_method = globals()[args.method]

  fig = plt.figure()
  ax = fig.add_subplot(111)
  plt.ion()  # Interactive mode.
  plt.grid('on')
  plt.axis('equal')
  plt.xlim([-0.5, 2])
  plt.ylim([-0.75, 1.25])
  plt.show()
  colors = colors_from('jet', len(args.dt))

  # Show all dt.
  for color, dt in zip(colors, args.dt):
    print('Using dt = {}'.format(dt))

    # Initial robot pose (x, y and theta).
    robot_pose = np.array([0., 0., 0.], dtype=np.float32)
    robot_drawer = RobotDrawer(ax, robot_pose, color=color, label='dt = %.3f [s]' % dt)
    if args.animate:
      fig.canvas.draw()
      fig.canvas.flush_events()

    # Simulate for 10 seconds.
    last_time_drawn = 0.
    last_time_drawn_real = time.time()
    for t in np.arange(0., 10., dt):
      robot_pose = integration_method(robot_pose, t, dt)
      
      plt.title('time = %.3f [s] with dt = %.3f [s]' % (t + dt, dt))
      robot_drawer.update(robot_pose)

      # Do not draw too many frames.
      time_drawn = t
      if args.animate and (time_drawn - last_time_drawn > REFRESH_RATE):
        # Try to draw in real-time.
        time_drawn_real = time.time()
        delta_time_real = time_drawn_real - last_time_drawn_real
        if delta_time_real < REFRESH_RATE:
          time.sleep(REFRESH_RATE - delta_time_real)
        last_time_drawn_real = time_drawn_real
        last_time_drawn = time_drawn
        fig.canvas.draw()
        fig.canvas.flush_events()
    robot_drawer.done()

  plt.ioff()
  plt.title('Trajectories')
  plt.legend(loc='lower right')
  plt.show(block=True)


# Simple class to draw and animate a robot.
class RobotDrawer(object):

  def __init__(self, ax, pose, radius=.05, label=None, color='g'):
    self._pose = pose.copy()
    self._radius = radius
    self._history_x = [pose[X]]
    self._history_y = [pose[Y]]
    self._outside = ax.plot([], [], 'b', lw=2)[0]
    self._front = ax.plot([], [], 'b', lw=2)[0]
    self._path = ax.plot([], [], c=color, lw=2, label=label)[0]
    self.draw()

  def update(self, pose):
    self._pose = pose.copy()
    self._history_x.append(pose[X])
    self._history_y.append(pose[Y])
    self.draw()

  def draw(self):
    a = np.linspace(0., 2 * np.pi, 20)
    x = np.cos(a) * self._radius + self._pose[X]
    y = np.sin(a) * self._radius + self._pose[Y]
    self._outside.set_data(x, y)
    r = np.array([0., self._radius])
    x = np.cos(self._pose[YAW]) * r + self._pose[X]
    y = np.sin(self._pose[YAW]) * r + self._pose[Y]
    self._front.set_data(x, y)
    self._path.set_data(self._history_x, self._history_y)

  def done(self):
    self._outside.set_data([], [])
    self._front.set_data([], [])


def colors_from(cmap_name, ncolors):
    cm = plt.get_cmap(cmap_name)
    cm_norm = matplotlib.colors.Normalize(vmin=0, vmax=ncolors - 1)
    scalar_map = matplotlib.cm.ScalarMappable(norm=cm_norm, cmap=cm)
    return [scalar_map.to_rgba(i) for i in range(ncolors)]


def positive_floats(string):
  values = tuple(float(v) for v in string.split(','))
  for v in values:
    if v <= 0.:
      raise argparse.ArgumentTypeError('{} is not strictly positive.'.format(v))
  return values


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Launches a battery of experiments in parallel')
  parser.add_argument('--method', action='store', default='euler', help='Integration method.', choices=['euler', 'rk4'])
  parser.add_argument('--dt', type=positive_floats, action='store', default=(0.05,), help='Integration step.')
  parser.add_argument('--animate', action='store_true', default=False, help='Whether to animate.')
  args = parser.parse_args()
  main(args)
