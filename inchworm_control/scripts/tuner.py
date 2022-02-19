#!/usr/bin/env python3

from math import pi
import rospy

from traj_planner import TrajectoryPlanner

def main():
  rospy.init_node("tuner")

  planner = TrajectoryPlanner()

  while True:
    # Get current joint state

    # Get desired joint to tune

    # Output current position, prompt for new position + duration

    # Run a non-blocking quintic trajectory

    # Collect data on desired vs actual position vs time

    # Plot desired/actual pos vs time on one plot, and err (desired - actual) vs time on another

    # Maybe prompt for new PID consts?

    # Rinse and repeat

    goals = [0] * 5

    for i in range(5):
      pos = float(input(f"Joint {i} position: "))

      # Eli can have his radians, I can have degrees
      if (abs(pos) > 2.2):
        pos = pos * pi / 180

      goals[i] = (pos)

    planner.run_quintic_traj(goals, 5.0)

if __name__ == "__main__":
  main()