#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


def braitenberg(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a braitenberg controller that takes the range
  # measurements given in argument to steer the robot.
  sigmoid_n = lambda z : (1/(1+np.exp(-z)))-0.5#0.5 to have a range from -0..5 to 0.5

 
  rad=1
  axl=2

  front=np.round(np.nan_to_num(front),4)
  #front=round(front,4)
  front_left=np.round(np.nan_to_num(front_left),4)
  front_right=np.round(np.nan_to_num(front_right),4)
  left=np.round(np.nan_to_num(left),4)
  right=np.round(np.nan_to_num(right),4)
  

  v_r_raw=np.tanh(front-1)*0.5 + np.tanh(front_left-1)*0.25 + np.tanh(left-1)*0.25
  v_l_raw=np.tanh(front-1)*0.5 + np.tanh(front_right-1)*0.25 + np.tanh(right-1)*0.25

  v_r=np.round(v_r_raw,3)
  v_l=np.round(v_l_raw,3)
  
  #coefs=np.array([[5,3,7],[1,2,5]])
  #dists=np.array([front, front_left, front_right, left,right]).reshape(5,1)
  #offsets=np.array([[-1],[-1]]).reshape(2,1)
    
  
  ### ROTATIONAL VELOCITY ###
  w=np.nan_to_num(rad/axl*(v_r-v_l))

    ### FORWARD VELOCITY ###
 # OFFSET=0
 # u=sigmoid_n(front)-OFFSET
  #u=np.nan_to_num((rad/2)*(v_r+v_l))
  u=np.nan_to_num(rad/2)*(v_r+v_l)
  
  print(u,'\t',w, v_r, v_l)
#guest editions
#instead use a minimum of abs 
  if(np.isnan(u)):
    print(front, front_left, front_right, left, right)
    u=0

  if(np.isnan(w)):
    w=0

  print(u, '\t',w)
 # print('front d,v', front, u, 'right d v', right, v_r, 'left d v', left, v_l)
  return u, w


def rule_based(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a rule-based controller that avoids obstacles.
  LIMIT=0.75
  
  REVERSE=-1
  SPEED_SLOW=0.1
  SPEED_NORMAL=0.5
  SPEED_FAST=1

  v_r=SPEED_NORMAL 
  v_l=SPEED_NORMAL
     
  if(front<LIMIT):  
     v_r=SPEED_FAST
     v_l=REVERSE
  elif (right<LIMIT):
     v_r=SPEED_FAST
     v_l=SPEED_SLOW
  elif (left<LIMIT):
     v_l=SPEED_FAST 
     v_r=SPEED_SLOW 
  elif (front_right<LIMIT):
       v_r=SPEED_FAST
       v_l=SPEED_NORMAL
  elif (front_left<LIMIT):
       v_l=SPEED_FAST 
       v_r=SPEED_NORMAL

                
  rad=1
  axl=1

      ### ROTATIONAL VELOCITY ###
  w=rad/axl*(v_r-v_l)

  ### FORWARD VELOCITY ###
  u=(rad/2)*(v_r+v_l)
  
  return u, w


class SimpleLaser(object):
  def __init__(self):
    rospy.Subscriber('/scan', LaserScan, self.callback)
    self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
    self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
    self._measurements = [float('inf')] * len(self._angles)
    self._indices = None

  def callback(self, msg):
    # Helper for angles.
    def _within(x, a, b):
      pi2 = np.pi * 2.
      x %= pi2
      a %= pi2
      b %= pi2
      if a < b:
        return a <= x and x <= b
      return a <= x or x <= b;

    # Compute indices the first time.
    if self._indices is None:
      self._indices = [[] for _ in range(len(self._angles))]
      for i, d in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        for j, center_angle in enumerate(self._angles):
          if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
            self._indices[j].append(i)

    ranges = np.array(msg.ranges)
    for i, idx in enumerate(self._indices):
      # We do not take the minimum range of the cone but the 10-th percentile for robustness.
      self._measurements[i] = np.percentile(ranges[idx], 10)

  @property
  def ready(self):
    return not np.isnan(self._measurements[0])

  @property
  def measurements(self):
    return self._measurements


class GroundtruthPose(object):
  def __init__(self, name='turtlebot3_burger'):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._name = name

  def callback(self, msg):
    idx = [i for i, n in enumerate(msg.name) if n == self._name]
    if not idx:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name))
    idx = idx[0]
    self._pose[0] = msg.pose[idx].position.x
    self._pose[1] = msg.pose[idx].position.y
    _, _, yaw = euler_from_quaternion([
        msg.pose[idx].orientation.x,
        msg.pose[idx].orientation.y,
        msg.pose[idx].orientation.z,
        msg.pose[idx].orientation.w])
    self._pose[2] = yaw

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose
  

def run(args):
  rospy.init_node('obstacle_avoidance')
  avoidance_method = globals()[args.mode]

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  laser = SimpleLaser()
  # Keep track of groundtruth position for plotting purposes.
  groundtruth = GroundtruthPose()
  pose_history = []
  with open('/tmp/gazebo_exercise.txt', 'w'):
    pass

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not laser.ready or not groundtruth.ready:
      rate_limiter.sleep()
      continue

    u, w = avoidance_method(*laser.measurements)
    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w
    publisher.publish(vel_msg)

    # Log groundtruth positions in /tmp/gazebo_exercise.txt
    pose_history.append(groundtruth.pose)
    if len(pose_history) % 10:
      with open('/tmp/gazebo_exercise.txt', 'a') as fp:
        fp.write('\n'.join(','.join(str(v) for v in p) for p in pose_history) + '\n')
        pose_history = []
    rate_limiter.sleep()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
  parser.add_argument('--mode', action='store', default='braitenberg', help='Method.', choices=['braitenberg', 'rule_based'])
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass
