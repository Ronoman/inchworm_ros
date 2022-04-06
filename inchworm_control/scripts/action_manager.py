#!/usr/bin/env python3

import rospy, actionlib

from inchworm_control.msg import InchwormAction

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
    
    if goal.action_type == 0:
      self.handleMove(goal)
    else:
      rospy.logerr(f"Action type {goal.action_type} not yet implemented.")

    self.server.set_succeeded()

  def handleMove(self, goal):
    '''
    Moves an inchworm to hover above a new shingle.
    If the end effector is in the air, then it will swing to hover over the target
    If the end effector is on a shingle, the inchworm will swap feet. WARNING: This assumes that the robot is in a good position to swap feet.
    '''

    # End effector is currently on the roof, so we need to swap it
    if (self.inchworm.foot_down == 0 and goal.end_effector == 0) or (self.inchworm.foot_down == 1 and goal.end_effector == 1):
      self.inchworm.swapFeet()
      rospy.sleep(0.25)

    # Offset to get from where inchworm is to the desired position
    coord_offset = (goal.coord_x - self.inchworm.on_coord[0], goal.coord_y - self.inchworm.on_coord[1])

    neighbor = None

    # Determine the neighbor to move to
    if self.inchworm.on_coord[1] % 2 == 0:
      neighbor = Inchworm.EVEN_ROW_LOOKUP_NEIGHBOR_MAP[coord_offset]
    else:
      neighbor = Inchworm.ODD_ROW_LOOKUP_NEIGHBOR_MAP[coord_offset]

    # Move to that neighbor
    self.inchworm.move(neighbor)

    rospy.sleep(0.5)

    self.server.set_succeeded()

def main():
  rospy.init_node("action_manager")

  inchworm_count = rospy.get_param("robot_count")

  servers = []

  for i in range(inchworm_count):
    servers.append(InchwormActionServer(i))

  rospy.loginfo("All inchworm action servers started")

if __name__ == "__main__":
  main()