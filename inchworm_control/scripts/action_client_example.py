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

  goal = InchwormGoal()

  # Move action
  goal.action_type = 0

  goal.coord_x = 1
  goal.coord_y = 1

  # Top end effector
  goal.end_effector = 1

  client_0.send_goal(goal)

  rospy.sleep(1)

  goal.coord_x = 2
  client_1.send_goal(goal)

  client_0.get_state()

  client_0.wait_for_result()
  client_1.wait_for_result()

  goal.coord_x = 0
  goal.coord_y = 2
  goal.end_effector = 0

  client_0.send_goal(goal)
  rospy.sleep(1)

  goal.coord_x = 1
  client_1.send_goal(goal)

  client_0.wait_for_result()
  client_1.wait_for_result()

  goal.coord_x = 0
  goal.action_type = 1
  client_0.send_goal(goal)
  client_0.wait_for_result()

if __name__ == "__main__":
  main()