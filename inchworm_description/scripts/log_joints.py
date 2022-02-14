#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState

last_state = None

def jointStateCB(msg):
  global last_state

  last_state = msg

def main():
  rospy.init_node("log_joints")

  rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB)

  while not rospy.is_shutdown():
    input()
    print(last_state)

if __name__ == "__main__":
  main()