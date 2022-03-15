#!/usr/bin/env python3

import rospy, math

from assembly_msgs.msg import MateList
from assembly_msgs.srv import SetMateSuppression, SetMateSuppressionRequest

from traj_planner import TrajectoryPlanner
from enum import Enum
from joint_consts import JointConstants

class Inchworm:
  class Neighbors(Enum):
    UPPER_LEFT = 1
    UPPER_RIGHT = 2
    RIGHT = 3
    BOTTOM_RIGHT = 4
    BOTTOM_LEFT = 5
    LEFT = 6

  def __init__(self, idx=0):
    self.idx = idx

    self.planner = TrajectoryPlanner(idx=self.idx)

    self.roof_height = rospy.get_param("/roof_height")
    self.roof_width  = rospy.get_param("/roof_width")

    # 0 is bottom foot, 1 is top foot
    self.foot_down = 0

    # Index of shingle the robot is on. Updated by mateCB from Magnet Sim
    self.on_shingle = 0

    # Suppression service proxy
    self.suprpess_proxy = rospy.ServiceProxy("/suppress_mate", SetMateSuppression)

    # State subscribers
    rospy.Subscriber("/active_mates", MateList, self.mateCB)

    # self.mag_state_pub = rospy.Publisher()

    rospy.logdebug(f"Initialize inchworm class for inchworm {self.idx}.")

  def moveTo(neighbor):
    '''
    Moves the end effector not currently attached to the roof to a neighbor.
    neighbor: One of the `Neighbors` enum. This gets mapped to a JointConstants set of joint angles
    '''
    
    pass

  def swapFeet():
    '''
    Swaps which foot is attached to the roof. Make sure the free end effector is placed above a mounting point before calling.
    '''

    pass

  def mateCB(self, msg):
    '''
    Callback for active_mate messages. Used to determine where the robot is currently
    '''

    # For all iw_root_N in the mates
    for i,mate in enumerate(msg.male):
      # If my index is in the string, we're mated to a shingle
      if str(self.idx) in mate:
        on_shingle = int(msg.female[i][-1])

        # If the robot has moved, trigger an update on suppressed mates
        if not on_shingle == self.on_shingle:
          self.updateSuppressedMates()

  def idx_to_coord(self, index, width):
    return (index % width, math.floor(index / width))

  def coord_to_idx(self, coord, width):
    return width*coord[1] + coord[0]

  def getAdjacentShingleIndexes(self, relative_to):
    '''
    Returns a list of shingle indexes that are adjacent to the robot. Used to determine which mates to disable
    '''
    
    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

    # Determine the shingle coords of the relative_to position
    coord = self.idx_to_coord(relative_to, self.roof_width)

    pair_offsets = []

    if coord[1] % 2 == 0:
      pair_offsets = ODD_ROW_N_LOOKUP
    else:
      pair_offsets = EVEN_ROW_N_LOOKUP

    neighbor_idx = []

    for pair in pair_offsets:
      adj_coord = self.coord_to_idx((coord[0] + pair[0], coord[1] + pair[1]), self.roof_width)
      if not adj_coord < 0:
        neighbor_idx.append(adj_coord)

    return neighbor_idx

  def updateSuppressedMates(self, currently_on, going_to):
    '''
    Suppresses all possible mates for shingles adjacent to `currently_on`, 
    unsuppresses all possible mates for shingles adjacent to `going_to`.
    '''
    
    adj_current = self.getAdjacentShingleIndexes(currently_on)
    adj_going   = self.getAdjacentShingleIndexes(going_to)

    # Suppress all in adjacent current
    for idx in adj_current:
      pass

    # Unsuppress all in where you're going to
    for idx in adj_going:
      pass

if __name__ == "__main__":
  # This only exists to test the class.

  rospy.init_node("iw_class_test")

  iw = Inchworm(idx=0)
  rospy.sleep(1)

  print(iw.getAdjacentShingleIndexes(iw.on_shingle))