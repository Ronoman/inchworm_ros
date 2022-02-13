#!/usr/bin/env python3

import rospy

import numpy as np

from sensor_msgs.msg import JointState
from inchworm_hw_interface.msg import MagnetState

# Lovingly ripped from 3001 code
def cubic_traj(t0, tf, p0, pf, v0, vf):
  M = np.array([[1, t0, t0**2, t0**3],
                [0, 1, 2*t0, 3*t0**2],
                [1, tf, tf**2, tf**3],
                [0, 1, 2*tf, 3*tf**2]])

  constraints = np.array([p0, v0, pf, vf])

  return np.matmul(np.linalg.inv(M), constraints)

def quintic_traj(t0, tf, p0, pf, v0, vf, a0, af):
  M = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

  constraints = np.array([p0, v0, a0, pf, vf, a0])

  return np.matmul(np.linalg.inv(M), constraints)

def main():
  rospy.init_node("smooth_traj")

  while not rospy.is_shutdown():
    duration = float(input("Traj duration (s): "))
    

if __name__ == "__main__":
  main()