#!/usr/bin/env python3

import rospy, math, sys, tf2_ros

from assembly_msgs.msg import MateList
from assembly_msgs.srv import SuppressMate, SuppressMateRequest, SuppressLink, SuppressLinkRequest

from traj_planner import TrajectoryPlanner
from enum import Enum
from joint_consts import JointConstants
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

  # Transform stuff
  def getTransform(frame_from, frame_to, buffer, listener):
      have_transform = False
      trans = None

      while not have_transform:
          try:
              trans = buffer.lookup_transform(frame_from, frame_to, rospy.Time(0))
              have_transform = True
          except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
              print(f"Lookup from {frame_from} to {frame_to} failed with error:")
              print(err)

              rospy.sleep(1.0)
              continue

      return trans

  def transToRPY(trans):
      quat = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]

      (r, p, y) = [math.degrees(n) for n in euler_from_quaternion(quat)]

      return r,p,y

  def rpyToQuat(roll, pitch, yaw):
      # Assume incoming is in degrees
      roll, pitch, yaw = [math.radians(n) for n in (roll, pitch, yaw)]

      quat = quaternion_from_euler(roll, pitch, yaw)

      return quat
  # Transform stuff done

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

  def moveTo(self,neighbor):
    '''
    Moves the end effector not currently attached to the roof to a neighbor.
    neighbor: One of the `Neighbors` enum. This gets mapped to a JointConstants set of joint angles
    '''
    jointMap = {
      1: JointConstants.upper_left,
      2: JointConstants.upper_right,
      3: JointConstants.right,
      4: JointConstants.bottom_right,
      5: JointConstants.bottom_left,
      6: JointConstants.left
    }
    if (neighbor == 0) : #for testing reasons
      angles = [0, 0, 0, 0, 0]
    elif (neighbor <= 6):
      angles = jointMap.get(neighbor)
    elif (neighbor == 7):
      a1 = 28.369 * math.pi / 180
      a2 = 123.27 * math.pi / 180
      a3 = 37.389 * math.pi / 180
      angles = [-2.4634861806310004, a1, a2, a3 ,2.46881244854237]
	

    # if self.foot_down == 1: #depending on the foot down, the joint angles swap
     # angles.reverse()

    self.planner.run_quintic_traj(angles, 4.0)
    #return when done?
    

  def swapMagnet(self, turnOff, turnOn):
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

      req.scoped_male = turnOff
      self.mate_suppress_proxy(req)

    req.suppress = False

    # Unsuppress all in where you're going to
    for idx in adj_going:
      print(f"\tUnsuppressing shingle {idx}")
      shingle = ["inchworm", f"shingle_description_{idx}", f"shingle_{idx}"]

      req.scoped_female = shingle

      req.scoped_male = turnOn
      self.mate_suppress_proxy(req)


  def swapFeet(self):
    '''
    Swaps which foot is attached to the roof. Make sure before calling the free end effector is placed above a mounting point before calling.
    '''
    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]    

    # self.currently_on = #TODO eli ????????

    if (self.foot_down == 0):
      turnOff = iw_bot
      turnOn = iw_top
      #switch to top foot
      self.foot_down = 1
      pass
    if (self.foot_down == 1):
      turnOff = iw_top
      turnOn = iw_bot
      #switch to bottom foot
      self.foot_down = 0
      pass

    self.swapMagnet(turnOff, turnOn)
    

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
          # TODO: Finish the logic here
          self.updateSuppressedMates()
          self.on_shingle = on_shingle

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

  #go to hover above upper-left
  iw.moveTo(1)
  rospy.sleep(5)
  #supress the top foot magnet
  #put the end effector down
  iw.moveTo(7)
  rospy.sleep(5)
  #turn on the top foot magnet
  #turn off the bottom foot magnet
  #switch planted foot
  iw.swapFeet()
  #straighten out
  rospy.sleep(1)
  iw.moveTo(0)

  rospy.sleep(5)