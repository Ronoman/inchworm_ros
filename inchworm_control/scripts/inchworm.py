#!/usr/bin/env python3

import rospy, math, sys

from assembly_msgs.msg import MateList
from assembly_msgs.srv import SuppressMate, SuppressMateRequest, SuppressLink, SuppressLinkRequest

from traj_planner import TrajectoryPlanner
from enum import Enum
from joint_consts import JointConstants

class Inchworm:
  '''
  High level controller for inchworms in the physics sim. Features:
  1. Manages mating and unmating with the magnet sim, including optimizations so that the sim runs at a reasonable speed
  2. swapFeet(): Swaps which foot is currently attached to the roof. Should be called when ready to swap which tile the inchworm is on.
  '''

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
    self.on_shingle = self.idx

    # Suppression service proxy
    rospy.wait_for_service("/suppress_mate")
    self.mate_suppress_proxy = rospy.ServiceProxy("/suppress_mate", SuppressMate)
    rospy.wait_for_service("/suppress_link")
    self.link_suppress_proxy = rospy.ServiceProxy("/suppress_link", SuppressLink)

    # State subscribers
    rospy.Subscriber("/active_mates", MateList, self.mateCB)

    # self.mag_state_pub = rospy.Publisher()

    print("Disabling all mates...")
    self.suppressAll()

    # This shouldn't disable any mates, and should enable mates adjacent to and including the starting shingle.
    print("Enabling starting point mates...")
    self.updateSuppressedMates(-9999999999, self.on_shingle)

    print(f"Initialize inchworm class for inchworm {self.idx}.")

  def suppressAll(self):
    '''
    Run on __init__(). Suppresses all robot mates. Immediately followed by a updateSuppressedMates() call to enable
    starting position mates.
    '''

    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]

    req = SuppressLinkRequest()
    req.suppress = True
    req.scoped_link = iw_bot

    self.link_suppress_proxy(req)
    
    req.scoped_link = iw_top
    self.link_suppress_proxy(req)


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
      pair_offsets = EVEN_ROW_N_LOOKUP
    else:
      pair_offsets = ODD_ROW_N_LOOKUP

    neighbor_coords = []

    for pair in pair_offsets:
      adj_coord = (coord[0] + pair[0], coord[1] + pair[1])
      if adj_coord[0] >= 0 and adj_coord[0] < self.roof_width and adj_coord[1] >= 0 and adj_coord[1] < self.roof_height:
        neighbor_coords.append(adj_coord)

    return [self.coord_to_idx(coord, self.roof_width) for coord in neighbor_coords]

  def updateSuppressedMates(self, currently_on, going_to):
    '''
    Suppresses all possible mates for shingles adjacent to `currently_on`, 
    unsuppresses all possible mates for shingles adjacent to `going_to`.
    '''
    
    # Find all shingle indices adjacent to and including currently_on and going_to.
    adj_current = self.getAdjacentShingleIndexes(currently_on) + [currently_on] if currently_on > -1 else []
    adj_going   = self.getAdjacentShingleIndexes(going_to) + [going_to]

    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]

    req = SuppressMateRequest()
    req.suppress = True

    # Suppress all in adjacent current
    for idx in adj_current:
      print(f"\tSuppressing shingle {idx}")
      shingle = ["inchworm", f"shingle_description_{idx}", f"shingle_{idx}"]

      req.scoped_female = shingle

      req.scoped_male = iw_bot
      self.mate_suppress_proxy(req)

      req.scoped_male = iw_top
      self.mate_suppress_proxy(req)

    req.suppress = False

    # Unsuppress all in where you're going to
    for idx in adj_going:
      print(f"\tUnsuppressing shingle {idx}")
      shingle = ["inchworm", f"shingle_description_{idx}", f"shingle_{idx}"]

      req.scoped_female = shingle

      req.scoped_male = iw_bot
      self.mate_suppress_proxy(req)

      req.scoped_male = iw_top
      self.mate_suppress_proxy(req)

if __name__ == "__main__":
  # This only exists to test the class.

  rospy.init_node("iw_class_test")

  iw = Inchworm(idx=0)
  rospy.sleep(1)

  print(iw.getAdjacentShingleIndexes(12))