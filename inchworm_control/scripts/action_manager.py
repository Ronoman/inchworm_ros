#!/usr/bin/env python3

import rospy, actionlib

from inchworm_control.msg import InchwormAction, InchwormResult

from inchworm import Inchworm

class InchwormActionServer:
  def __init__(self, idx=0):
    self.server = actionlib.SimpleActionServer(f"inchworm_action_{idx}", InchwormAction, self.execute, False)
    
    self.inchworm = Inchworm(idx=idx)

    self.server.start()
  
  def execute(self, goal):
    '''
    Dispatches an InchwormActionGoal. Runs in a thread (whatever that means in python), so multiple servers can handle execute at the same time.
    '''
    rospy.loginfo(f"IW Action Server {self.inchworm.idx} received goal:")
    print(goal)

    success = False
    
    if goal.action_type == 0:
      success = self.handleMove(goal)
    elif goal.action_type == 1:
      success = self.handlePick(goal)
    elif goal.action_type == 2:
      success = self.handlePlace(goal)
    else:
      rospy.logerr(f"Action type {goal.action_type} not yet implemented.")

    self.server.set_succeeded(result=InchwormResult(success=success))

  def handleMove(self, goal):
    '''
    Moves an inchworm to hover above a new shingle.
    If the end effector is in the air, then it will swing to hover over the target
    If the end effector is on a shingle, the inchworm will swap feet. WARNING: This assumes that the robot is in a good position to swap feet.
    '''

    # End effector is currently on the roof, so we need to swap it
    if (self.inchworm.foot_down == 0 and goal.end_effector == 0) or (self.inchworm.foot_down == 1 and goal.end_effector == 1):
      self.inchworm.swapFeet()
      rospy.sleep(2)

    neighbor = self.inchworm.absoluteToNeighbor((goal.coord_x, goal.coord_y))

    # Move to that neighbor
    self.inchworm.move(neighbor, plantFoot=False)

    rospy.sleep(0.5)

    return True

  def handlePick(self, goal):
    '''
    Picks up a shingle. End effector must be over the desired shingle.
    '''

    neighbor = self.inchworm.absoluteToNeighbor((goal.coord_x, goal.coord_y))
    self.inchworm.pickupShingle(neighbor)

    return True

  def handlePlace(self, goal):
    '''
    Places a shingle. End effector must be over the desired roof mount point.
    '''

    neighbor = self.inchworm.absoluteToNeighbor((goal.coord_x, goal.coord_y))
    self.inchworm.placeShingle(neighbor)

    return True

def main():
  rospy.init_node("action_manager")

  inchworm_count = rospy.get_param("robot_count")

  servers = []

  for i in range(inchworm_count):
    servers.append(InchwormActionServer(i))

  rospy.loginfo("All inchworm action servers started")

if __name__ == "__main__":
  main()