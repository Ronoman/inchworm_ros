#!/usr/bin/env python3

import rospy

from threading import Lock

from sensor_msgs.msg import JointState

current_goal = None

goal_lock = Lock()

def goalCB(msg):
  global current_goal
  
  goal_lock.acquire()
  current_goal = msg
  goal_lock.release()

def main():
  global current_goal

  rospy.Subscriber("/hw_interface/joint_goal", JointState, goalCB)
  goal_pub = rospy.Publisher("current_goal", JointState, queue_size=10)

  goal_lock.acquire()
  current_goal = JointState()
  current_goal.name = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]
  current_goal.position = [0]*5
  current_goal.velocity = [0]*5
  current_goal.effort   = [0]*5
  goal_lock.release()

  rate = rospy.Rate(rospy.get_param("publish_rate", 25))

  while not rospy.is_shutdown():
    goal_lock.acquire()
    current_goal.header.stamp = rospy.Time.now()
    goal_pub.publish(current_goal)
    goal_lock.release()

    rate.sleep()

if __name__ == "__main__":
  rospy.init_node("goal_tracker")

  main()