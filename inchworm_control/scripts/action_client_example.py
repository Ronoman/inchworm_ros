#!/usr/bin/env python3

import rospy, actionlib

from inchworm_control.msg import InchwormAction, InchwormGoal

def main():
  rospy.init_node("action_client_example")

  client_0 = actionlib.SimpleActionClient("/inchworm_action_0", InchwormAction)
  client_0.wait_for_server()

  client_1 = actionlib.SimpleActionClient("/inchworm_action_1", InchwormAction)
  client_1.wait_for_server()

  rospy.loginfo("Got servers, sending goals.")

  goal0 = InchwormGoal()
  goal1 = InchwormGoal()

  # Move action
  goal0.action_type = 0
  goal0.coord_x = 1
  goal0.coord_y = 2
  # Top end effector
  goal0.end_effector = 1

  client_0.send_goal(goal0)

  rospy.sleep(1)

  goal1.action_type = 0
  goal1.coord_x = 1
  goal1.coord_y = 0
  goal1.end_effector = 1
  client_1.send_goal(goal1)

  client_0.get_state()

  client_0.wait_for_result()
  client_1.wait_for_result()

  # Move action
  goal0.action_type = 0
  goal0.coord_x = 2
  goal0.coord_y = 1
  # Top end effector
  goal0.end_effector = 0

  client_0.send_goal(goal0)

  rospy.sleep(1)

  goal1.action_type = 0
  goal1.coord_x = 0
  goal1.coord_y = 0
  goal1.end_effector = 0
  client_1.send_goal(goal1)

  client_0.get_state()

  client_0.wait_for_result()
  client_1.wait_for_result()


 

if __name__ == "__main__":
  main()