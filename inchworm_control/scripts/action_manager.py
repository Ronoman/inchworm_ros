#!/usr/bin/env python3

import rospy, actionlib, os

from inchworm_control.msg import InchwormAction, InchwormResult

from inchworm import Inchworm
from shingle_manager import ShingleManager

from rospkg import RosPack

from threading import Lock

class InchwormActionServer:
  def __init__(self, idx, manager, log_file=None, run_count=0):
    self.server = actionlib.SimpleActionServer(f"inchworm_action_{idx}", InchwormAction, self.execute, False)
    
    self.idx = idx

    self.inchworm = Inchworm(idx=idx)
    self.manager = manager

    self.log_file = log_file

    if log_file is None:
      self.log_file = os.path.join(RosPack().get_path("inchworm_control"), f"logs/{run_count}/iw_{idx}.log")

    self.server.start()

  def logAction(self, goal):
    rospy.loginfo(self.log_file)
    with open(self.log_file, "a+") as f:
      f.write(f"-----\n")
      f.write(f"currently on shingle {self.inchworm.on_shingle}, coord {self.inchworm.on_coord}\n")
      f.write(("bottom" if self.inchworm.foot_down == False else "top") + " EE is down\n")
      f.write(f"-\n")
      f.write(f"timestamp: {rospy.Time.now().to_time()}\n")
      f.write(str(goal) + "\n\n")
  
  def execute(self, goal):
    '''
    Dispatches an InchwormActionGoal. Runs in a thread (whatever that means in python), so multiple servers can handle execute at the same time.
    '''
    rospy.loginfo(f"IW Action Server {self.inchworm.idx} received goal:")
    rospy.loginfo(goal)
    self.logAction(goal)

    success = False
    
    if goal.action_type == 0:
      success = self.handleMove(goal)
    elif goal.action_type == 1:
      success = self.handlePick(goal)
    elif goal.action_type == 2:
      success = self.handlePlace(goal)
    elif goal.action_type == 3:
      success = self.handleSpawn(goal)
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

  def handleSpawn(self, goal):
    '''
    Spawns a shingle. 
    '''

    self.manager.spawnShingle((goal.coord_x, goal.coord_y))

    return True

def main():
  rospy.init_node("action_manager")

  inchworm_count = rospy.get_param("robot_count")
  width = rospy.get_param("/roof_width")
  height = rospy.get_param("/roof_height")

  servers = []

  run_count = 0
  run_count_path = os.path.join(RosPack().get_path("inchworm_control"), "logs/run_count.txt")
  with open(run_count_path, "r") as f:
    run_count = f.readline().strip("\n")
  with open(run_count_path, "w") as f:
    f.write(str(int(run_count) + 1))

  os.mkdir(os.path.join(RosPack().get_path("inchworm_control"), "logs", str(run_count)))

  manager = ShingleManager(width, height)

  for i in range(inchworm_count):
    servers.append(InchwormActionServer(i, manager, None, int(run_count)))

  rospy.loginfo("All inchworm action servers started")

if __name__ == "__main__":
  main()