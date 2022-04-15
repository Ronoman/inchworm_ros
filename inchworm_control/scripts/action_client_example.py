#!/usr/bin/env python3

import rospy, actionlib

from inchworm_control.msg import InchwormAction, InchwormGoal

def spawnShingles(spawn, client_0):
  spawn.action_type = 3
  spawn.coord_x = 0
  spawn.coord_y = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 3
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 4
  client_0.send_goal(spawn)
  client_0.wait_for_result()


  spawn.coord_x = 0
  spawn.coord_y = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 3
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 4
  client_0.send_goal(spawn)
  client_0.wait_for_result()

  #spawn.coord_x = 0
  #spawn.coord_y = 3
  #client_0.send_goal(spawn)
  #client_0.wait_for_result()
  #spawn.coord_x = 1
  #client_0.send_goal(spawn)
  #client_0.wait_for_result()
  #spawn.coord_x = 2
  #client_0.send_goal(spawn)
  #client_0.wait_for_result()
  # spawn.coord_x = 3
  # client_0.send_goal(spawn)
  # client_0.wait_for_result()
  # spawn.coord_x = 4
  # client_0.send_goal(spawn)
  # client_0.wait_for_result()


def walk0(goal0, x, y, ee):
  goal0.action_type = 0
  goal0.coord_x = x
  goal0.coord_y = y
  # Top end effector
  goal0.end_effector = ee


def walk1(goal1, x, y, ee):
  goal1.action_type = 0
  goal1.coord_x = x
  goal1.coord_y = y
  # Top end effector
  goal1.end_effector = ee


def main():
  rospy.init_node("action_client_example")

  client_0 = actionlib.SimpleActionClient("/inchworm_action_0", InchwormAction)
  client_0.wait_for_server()

  client_1 = actionlib.SimpleActionClient("/inchworm_action_1", InchwormAction)
  client_1.wait_for_server()

  rospy.loginfo("Got servers, sending goals.")

  

  goal0 = InchwormGoal()
  goal1 = InchwormGoal()
  spawn = InchwormGoal()

  spawnShingles(spawn, client_0)

  walk0(goal0, 0, 1, 1)
  walk1(goal1, 2, 0, 1)
  client_0.send_goal(goal0)
  client_1.send_goal(goal1)
  client_0.wait_for_result()
  client_1.wait_for_result()


  walk0(goal0, 0, 2, 0)
  walk1(goal1, 3, 0, 0)
  client_0.send_goal(goal0)
  client_1.send_goal(goal1)
  client_0.wait_for_result()
  client_1.wait_for_result()

  walk0(goal0, 1, 1, 1)
  walk1(goal1, 4, 0, 1)
  client_0.send_goal(goal0)
  client_1.send_goal(goal1)
  client_0.wait_for_result()
  client_1.wait_for_result()

  walk0(goal0, 2, 1, 0)
  walk1(goal1, 3, 0, 0)
  client_0.send_goal(goal0)
  client_1.send_goal(goal1)
  client_0.wait_for_result()
  client_1.wait_for_result()

  walk0(goal0, 3, 1, 1)
  walk1(goal1, 2, 0, 1)
  client_0.send_goal(goal0)
  client_1.send_goal(goal1)
  client_0.wait_for_result()
  client_1.wait_for_result()
 
  walk0(goal0, 4, 1, 0)
  walk1(goal1, 1, 0, 0)
  client_0.send_goal(goal0)
  client_1.send_goal(goal1)
  client_0.wait_for_result()
  client_1.wait_for_result()

  

if __name__ == "__main__":
  main()