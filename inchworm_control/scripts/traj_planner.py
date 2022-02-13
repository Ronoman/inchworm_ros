#!/usr/bin/env python3

import numpy as np

# This is a library of trajectory generation functions.

class TrajectoryPlanner:
  @staticmethod
  def cubic_traj(t0, tf, p0, pf, v0, vf):
    M = np.array([[1, t0, t0**2, t0**3],
                  [0, 1, 2*t0, 3*t0**2],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*tf, 3*tf**2]])

    constraints = np.array([p0, v0, pf, vf])

    return np.matmul(np.linalg.inv(M), constraints)

  @staticmethod
  def eval_cubic_pose(coeffs, t):
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3

  @staticmethod
  def eval_cubic_vel(coeffs, t):
    return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2

  @staticmethod
  def eval_cubic_accel(coeffs, t):
    return 2*coeffs[2] + 6*coeffs[3]*t

  @staticmethod
  def quintic_traj(t0, tf, p0, pf, v0, vf, a0, af):
    M = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                  [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                  [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                  [1, tf, tf**2, tf**3, tf**4, tf**5],
                  [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                  [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

    constraints = np.array([p0, v0, a0, pf, vf, af])

    return np.matmul(np.linalg.inv(M), constraints)

  @staticmethod
  def eval_quintic_pose(coeffs, t):
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5

  @staticmethod
  def eval_quintic_vel(coeffs, t):
    return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4

  @staticmethod
  def eval_quintic_accel(coeffs, t):
    return 2*coeffs[2]*t + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3

  @staticmethod
  def cubic_interp(t, p0, pf, v0, vf, num_pts):
    '''
    Generates cubic trajectory coefficients, then returns points, velocities, and accelerations interpolated along that curve. Includes endpoint
    '''

    coeffs = TrajectoryPlanner.cubic_traj(0, t, p0, pf, v0, vf)

    pts = np.linspace(0, t, num_pts)

    positions = [TrajectoryPlanner.eval_cubic_pose(coeffs, pt) for pt in pts]
    velocities = [TrajectoryPlanner.eval_cubic_vel(coeffs, pt) for pt in pts]
    accelerations = [TrajectoryPlanner.eval_cubic_accel(coeffs, pt) for pt in pts]

    return (positions, velocities, accelerations)

  @staticmethod
  def quintic_interp(t, p0, pf, v0, vf, a0, af, num_pts):
    '''
    Generates cubic trajectory coefficients, then returns points, velocities, and accelerations interpolated along that curve. Includes endpoint
    '''

    coeffs = TrajectoryPlanner.quintic_traj(0, t, p0, pf, v0, vf, a0, af)

    pts = np.linspace(0, t, num_pts)

    positions = [TrajectoryPlanner.eval_quintic_pose(coeffs, pt) for pt in pts]
    velocities = [TrajectoryPlanner.eval_quintic_vel(coeffs, pt) for pt in pts]
    accelerations = [TrajectoryPlanner.eval_quintic_accel(coeffs, pt) for pt in pts]

    return (positions, velocities, accelerations)

if __name__ == "__main__":
  # No need to import if this is run as a library
  from matplotlib import pyplot as plt

  # Simple test cases
  coeffs = TrajectoryPlanner.quintic_traj(0, 1, 10, -20, 0, 0, 0, 0)

  times = np.linspace(0, 1, 50)
  pts = TrajectoryPlanner.quintic_interp(1, 10, -20, 0, 0, 0, 0, 50)[2]

  plt.scatter(times, pts)
  plt.show()