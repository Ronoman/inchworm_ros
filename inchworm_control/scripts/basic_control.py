#!/usr/bin/env python3

from cmath import pi
import rospy, math, sys, tf2_ros

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sensor_msgs.msg import JointState

from traj_planner import TrajectoryPlanner

trajectory_pub   = None
mag_state_pub    = None
last_joint_state = None

def jointStateCB(msg):
    global last_joint_state

    last_joint_state = msg

def joint(joint_angles, duration):
  last_states = last_joint_state

  joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]
  cur_angles = []

  # Reorder the joint names to be the order specified by joint_names
  for name in joint_names:
    cur_angles.append(last_states.position[last_states.name.index(name)])

  # The desired angles from the payload
  new_angles = [float(q) for q in joint_angles]

  NUM_PTS = 50

  # Calculate the quintic trajectory for each joint. Impose 0 velocity and 0 acceleration at limits.
  traj_triplets = []
  for (p0, pf) in zip(cur_angles, new_angles):
    traj = TrajectoryPlanner.quintic_interp(duration, p0, pf, 0, 0, 0, 0, NUM_PTS)
    traj_triplets.append(traj)

  traj_pts = []

  # 
  for i in range(NUM_PTS):
    pt = JointTrajectoryPoint()
    pt.positions     = [traj_triplets[joint][0][i] for joint in range(5)]
    pt.velocities    = [traj_triplets[joint][1][i] for joint in range(5)]
    pt.accelerations = [traj_triplets[joint][2][i] for joint in range(5)]

    pt.time_from_start = rospy.Duration(i * (duration / NUM_PTS))

    traj_pts.append(pt)

  trajectory = JointTrajectory()
  trajectory.joint_names = joint_names
  trajectory.points = traj_pts

  trajectory.header.stamp = rospy.Time.now()
  trajectory.header.frame_id = "world"

  trajectory_pub.publish(trajectory)

  rospy.sleep(duration)


if __name__ == "__main__":
    rospy.init_node("basic_control")

    rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB)
    trajectory_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)

    while True:
        
        goals = [0] * 5

        for i in range(5):

            pos = float(input(f"Joint {i} position: "))
            if (abs(pos) > 2.2): # Eli can have his radians, I can have degrees
                pos = pos * pi / 180
            goals[i] = (pos)

        joint(goals, (5.0))