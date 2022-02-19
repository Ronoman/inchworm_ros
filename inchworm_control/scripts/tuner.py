#!/usr/bin/env python3

from math import pi
import rospy

from matplotlib import pyplot as plt

from traj_planner import TrajectoryPlanner
from control_msgs.msg import JointTrajectoryControllerState

cur_goal = None

def goalCB(msg):
  global cur_goal

  cur_goal = msg

def main():
  global cur_goal

  rospy.init_node("tuner")

  rospy.Subscriber("/inchworm/position_trajectory_controller/state", JointTrajectoryControllerState, goalCB)

  planner = TrajectoryPlanner()

  rospy.sleep(2.0)

  while True:
    # Get current joint state
    cur_state = planner.get_joint_state()

    # Get desired joint to tune
    joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]
    print("Joint names:")
    for i,joint in enumerate(joint_names):
      print(f"\tNum: {i}\tName: {joint}")

    joint_idx = int(input("Motor to test (num): "))

    # Output current position, prompt for new position + duration
    state_idx = cur_state.name.index(joint_names[joint_idx])
    print(f"Joint {joint_idx} pos (rad): {cur_state.position[state_idx]}")

    new_pos = float(input("New joint pos (rad): "))
    duration = float(input("Duration (s): "))

    input("Press enter to run trajectory.")

    # Run a non-blocking quintic trajectory
    angles = list(cur_state.position)
    print(angles)

    angles[state_idx] = new_pos
    
    reordered_angles = []

    for name in joint_names:
      reordered_angles.append(angles[cur_state.name.index(name)])

    print(reordered_angles)

    planner.run_quintic_traj(reordered_angles, duration, wait=False)

    # Collect data on desired vs actual position vs time
    start = rospy.Time.now()

    times = []
    desired = []
    actual = []
    error = []

    # While we are running the trajectory
    while rospy.Time.now() - start < rospy.Duration(duration):
      now = rospy.Time.now()

      times.append((now - start).nsecs + (now - start).secs * 10**9)

      desired_pos = cur_goal.desired.positions
      actual_pos = cur_goal.actual.positions
      error_pos = cur_goal.error.positions

      names = cur_goal.joint_names

      desired.append(desired_pos[names.index(joint_names[joint_idx])])
      actual.append(actual_pos[names.index(joint_names[joint_idx])])
      error.append(error_pos[names.index(joint_names[joint_idx])])

      rospy.sleep(0.01)

    # Plot desired/actual pos vs time on one plot, and err (desired - actual) vs time on another
    fig, axs = plt.subplots(2)
    fig.suptitle("Desired, Actual, and Error over time")

    axs[0].scatter(times, desired, label="Desired", s=1)
    axs[0].scatter(times, actual, label="Actual", s=1)
    axs[0].set_ylabel("Pos (rad)")
    axs[0].legend()
    
    axs[1].scatter(times, error, label="Error", s=1)
    axs[1].set_ylabel("Pos (rad)")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()

    plt.show()

if __name__ == "__main__":
  main()