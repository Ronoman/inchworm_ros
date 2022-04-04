#!/usr/bin/env python3

from audioop import reverse
from tkinter import LEFT
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
    NONE = 0 #for when the robot is straight up for motion planning, ignore for shingle
    UPPER_LEFT = 1
    UPPER_RIGHT = 2
    RIGHT = 3
    BOTTOM_RIGHT = 4
    BOTTOM_LEFT = 5
    LEFT = 6

  class Poses(Enum):
    STRAIGHT = 0
    UPPER_LEFT_DOWN = 1
    UPPER_LEFT_HOVER = 2
    UPPER_LEFT_LIFT = 3
    UPPER_RIGHT_DOWN = 4
    UPPER_RIGHT_HOVER = 5
    UPPER_RIGHT_LIFT = 6
    RIGHT_DOWN = 7
    RIGHT_HOVER = 8
    RIGHT_LIFT = 9
    BOTTOM_RIGHT_DOWN = 10
    BOTTOM_RIGHT_HOVER = 11
    BOTTOM_RIGHT_LIFT = 12
    BOTTOM_LEFT_DOWN = 13
    BOTTOM_LEFT_HOVER = 14
    BOTTOM_LEFT_LIFT = 15
    LEFT_DOWN = 16
    LEFT_HOVER = 17
    LEFT_LIFT = 18

  mapPoses = {
    Poses.STRAIGHT: Poses.STRAIGHT,
    Poses.UPPER_LEFT_DOWN: Poses.BOTTOM_RIGHT_DOWN,
    Poses.UPPER_LEFT_HOVER: Poses.BOTTOM_RIGHT_HOVER,
    Poses.UPPER_LEFT_LIFT: Poses.BOTTOM_RIGHT_LIFT,
    Poses.UPPER_RIGHT_DOWN: Poses.BOTTOM_LEFT_DOWN,
    Poses.UPPER_RIGHT_HOVER: Poses.BOTTOM_LEFT_HOVER,
    Poses.UPPER_RIGHT_LIFT: Poses.BOTTOM_LEFT_LIFT,
    Poses.RIGHT_DOWN: Poses.LEFT_DOWN,
    Poses.RIGHT_HOVER: Poses.LEFT_HOVER,
    Poses.RIGHT_LIFT: Poses.LEFT_LIFT,
    Poses.BOTTOM_RIGHT_DOWN: Poses.UPPER_LEFT_DOWN,
    Poses.BOTTOM_RIGHT_HOVER: Poses.UPPER_LEFT_HOVER,
    Poses.BOTTOM_RIGHT_LIFT: Poses.UPPER_LEFT_LIFT,
    Poses.BOTTOM_LEFT_DOWN: Poses.UPPER_RIGHT_DOWN,
    Poses.BOTTOM_LEFT_HOVER: Poses.UPPER_RIGHT_HOVER,
    Poses.BOTTOM_LEFT_LIFT: Poses.UPPER_RIGHT_LIFT,
    Poses.LEFT_DOWN: Poses.RIGHT_DOWN,
    Poses.LEFT_HOVER: Poses.RIGHT_HOVER,
    Poses.LEFT_LIFT: Poses.RIGHT_LIFT
  }

  jointMap = {
    Poses.STRAIGHT: JointConstants.straight,

    Poses.UPPER_LEFT_HOVER: JointConstants.upper_left,
    Poses.UPPER_LEFT_DOWN: JointConstants.upper_left_down,
    Poses.UPPER_LEFT_LIFT: JointConstants.upper_left_lift,

    Poses.UPPER_RIGHT_HOVER: JointConstants.upper_right,
    Poses.UPPER_RIGHT_DOWN: JointConstants.upper_right_down,
    Poses.UPPER_RIGHT_LIFT: JointConstants.upper_right_lift,

    Poses.RIGHT_HOVER: JointConstants.right,
    Poses.RIGHT_DOWN: JointConstants.right_down,
    Poses.RIGHT_LIFT: JointConstants.right_lift,

    Poses.BOTTOM_RIGHT_HOVER: JointConstants.bottom_right,
    Poses.BOTTOM_RIGHT_DOWN: JointConstants.bottom_right_down,
    Poses.BOTTOM_RIGHT_LIFT: JointConstants.bottom_right_lift,

    Poses.BOTTOM_LEFT_HOVER: JointConstants.bottom_left,
    Poses.BOTTOM_LEFT_DOWN: JointConstants.bottom_left_down,
    Poses.BOTTOM_LEFT_LIFT: JointConstants.bottom_left_lift,

    Poses.LEFT_HOVER: JointConstants.left,
    Poses.LEFT_DOWN: JointConstants.left_down,
    Poses.LEFT_LIFT: JointConstants.left_lift
  }

  transform = {
    Neighbors.LEFT: Neighbors.RIGHT,
    Neighbors.UPPER_LEFT: Neighbors.BOTTOM_RIGHT,
    Neighbors.UPPER_RIGHT: Neighbors.BOTTOM_LEFT,
    Neighbors.RIGHT: Neighbors.LEFT,
    Neighbors.BOTTOM_RIGHT: Neighbors.UPPER_LEFT,
    Neighbors.BOTTOM_LEFT: Neighbors.UPPER_RIGHT
  }

  mapNeighborToPoses = {
      Neighbors.BOTTOM_LEFT: [Poses.BOTTOM_LEFT_HOVER, Poses.BOTTOM_LEFT_LIFT, Poses.BOTTOM_LEFT_DOWN],
      Neighbors.BOTTOM_RIGHT:  [Poses.BOTTOM_RIGHT_HOVER, Poses.BOTTOM_RIGHT_LIFT, Poses.BOTTOM_RIGHT_DOWN],
      Neighbors.RIGHT:  [Poses.RIGHT_HOVER, Poses.RIGHT_LIFT, Poses.RIGHT_DOWN],
      Neighbors.UPPER_RIGHT:  [Poses.UPPER_RIGHT_HOVER, Poses.UPPER_RIGHT_LIFT, Poses.UPPER_RIGHT_DOWN],
      Neighbors.UPPER_LEFT:  [Poses.UPPER_LEFT_HOVER, Poses.UPPER_LEFT_LIFT, Poses.UPPER_LEFT_DOWN],
      Neighbors.LEFT:  [Poses.LEFT_HOVER, Poses.LEFT_LIFT, Poses.LEFT_DOWN],
      Neighbors.NONE: [Poses.STRAIGHT, Poses.STRAIGHT, Poses.STRAIGHT]
    }


  def __init__(self, idx=0):
    self.idx = idx

    self.planner = TrajectoryPlanner(idx=self.idx)

    self.roof_height = rospy.get_param("/roof_height")
    self.roof_width  = rospy.get_param("/roof_width")

    # 0 is bottom foot, 1 is top foot
    self.foot_down = 0

    # Index of shingle the robot is on. Updated by mateCB from Magnet Sim.
    self.on_shingle = -1

    #position on the roof of the robot
    #self.position = self.idx

    #for higher abstraction, last neighbor sent
    self.lastNeighbor = self.Neighbors.NONE
    # The roof coordinate that the robot is currently on. Roof idx != shingle idx. Updated by mateCB
    self.on_coord = self.idx_to_coord(self.idx, self.roof_width)

    # Most recent mate callback. Used to inspect current shingle config
    self.last_mate_msg = None

    # Suppression service proxy
    rospy.wait_for_service("/suppress_mate")
    self.mate_suppress_proxy = rospy.ServiceProxy("/suppress_mate", SuppressMate)
    rospy.wait_for_service("/suppress_link")
    self.link_suppress_proxy = rospy.ServiceProxy("/suppress_link", SuppressLink)

    # State subscribers
    rospy.Subscriber("/active_mates", MateList, self.mateCB)

    rospy.sleep(0.25)

    # self.mag_state_pub = rospy.Publisher()

    print("Disabling all mates...")
    self.suppressAll()

    # This shouldn't disable any mates, and should enable mates adjacent to and including the starting shingle.
    print("Enabling starting point mates...")
    self.updateSuppressedMates(-9999999999, self.coord_to_idx(self.on_coord, self.roof_width))

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

  def moveTo(self,pose,time):
    '''
    Moves the end effector not currently attached to the roof to a neighbor.
    neighbor: One of the `Neighbors` enum. This gets mapped to a JointConstants set of joint angles
    '''

    newPose = pose # if foot down is 0
    if self.foot_down == 1: #if foot down is 1, do the mapping
      newPose = self.mapPoses.get(pose)

    angles = self.jointMap.get(newPose)

    self.planner.run_quintic_traj(angles, time)

  def swapMagnet(self, turnOff, turnOn):
    # Find all shingle indices adjacent to and including currently_on and going_to.
    adj_current = self.getAdjacentShingleIndexes(self.idx_to_coord(self.on_shingle, self.roof_width)) + [self.on_shingle] if self.on_shingle > -1 else []

   

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

    # Unsuppress all in where you're going to (this is wrong, but this is for testing)
    for idx in adj_current:
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

    print(f"foot down: {self.foot_down}")
    print(f"on_shingle: {self.on_shingle}")
    if (self.foot_down == 0):
      turnOff = iw_bot
      turnOn = iw_top
      #switch to top foot
      foot_down = 1
      pass
    if (self.foot_down == 1):
      turnOff = iw_top
      turnOn = iw_bot
      #switch to bottom foot
      foot_down = 0
      pass

    self.foot_down = foot_down
    self.swapMagnet(turnOff, turnOn)

  def mateCB(self, msg):
    '''
    Callback for active_mate messages. Used to determine where the robot is currently
    '''

    self.last_mate_msg = msg
    
    # For all iw_root_N in the mates
    for i,mate in enumerate(msg.male):
      # If my index is in the string, we're mated to a shingle
      if f"inchworm_description_{self.idx}" in mate:
        # Fully scoped shingle names come in this format:
        # inchworm::shingle_description_{idx}::shingle_{idx}::1
        # Following line is to extract the first idx.
        on_shingle = int(msg.female[i].split("::")[1].split("_")[-1])

        # If the robot has moved, trigger an update on suppressed mates
        if not on_shingle == self.on_shingle:
          # Check if shingle is attached to roof
          for j,mate in enumerate(msg.female):
            if f"shingle_description_{on_shingle}" in mate:
              male_mate = msg.male[j]
              if f"roof_description" in male_mate:
                # Shingle is on the roof. Update on_shingle and on_coord
                print(f"before: {self.on_shingle} after: {on_shingle}")
                self.on_shingle = on_shingle
                roof_mate_idx = int(msg.male[j].split("::")[-1])
                print(f"Robot on roof index {roof_mate_idx}")
                self.on_coord = self.idx_to_coord(roof_mate_idx, self.roof_width)
                print(f"Robot on coord {self.on_coord}")

                return

  def idx_to_coord(self, index, width):
    return (index % width, math.floor(index / width))

  def coord_to_idx(self, coord, width):
    return width*coord[1] + coord[0]

  def getAdjacentShingleIndexes(self, roof_coord):
    '''
    Returns a list of shingle indexes that are adjacent to the robot. Used to determine which mates to disable
    This function uses self.on_coord, looks up adjacent roof indexes, then finds shingles that are attached to those roof mount points.
    '''
    
    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

    # The coordinate we look up relative to is the roof coordinate that the inchworm is on

    pair_offsets = []

    # Determine which list to lookup from, based on whether this inchworm is on an even or odd row
    if roof_coord[1] % 2 == 0:
      pair_offsets = EVEN_ROW_N_LOOKUP
    else:
      pair_offsets = ODD_ROW_N_LOOKUP

    neighbor_coords = []

    # If the coordinate is in bounds, append it to the list
    for pair in pair_offsets:
      adj_coord = (roof_coord[0] + pair[0], roof_coord[1] + pair[1])
      if adj_coord[0] >= 0 and adj_coord[0] < self.roof_width and adj_coord[1] >= 0 and adj_coord[1] < self.roof_height:
        neighbor_coords.append(adj_coord)

    # Convert all adjacent coordinates into indexes, so that we can lookup into the mate message
    roof_indexes = [self.coord_to_idx(coord, self.roof_width) for coord in neighbor_coords]

    # Shingle indexes attached to the roof_indexes above
    shingle_indexes = []

    while self.last_mate_msg is None:
      rospy.sleep(0.1)

    last_mate = self.last_mate_msg
    for idx in roof_indexes:
      for i,mate in enumerate(last_mate.male):
        # If we find the corresponding roof mate point in the male list, lookup the shingle index and add it to the output list
        if idx == int(mate.split("::")[-1]):
          shingle_idx = int(last_mate.female[i].split("::")[1].split("_")[-1])
          shingle_indexes.append(shingle_idx)

    print(f"Shingle indexes adjacent to roof coord {roof_coord}:")
    for idx in shingle_indexes:
      print(f"\t{idx}")

    return shingle_indexes

  def updateSuppressedMates(self, currently_on, going_to):
    '''
    Suppresses all possible mates for shingles adjacent to `currently_on`, 
    unsuppresses all possible mates for shingles adjacent to `going_to`.
    '''
    
    # Find all shingle indices adjacent to and including currently_on and going_to.
    adj_current = self.getAdjacentShingleIndexes(self.idx_to_coord(currently_on, self.roof_width)) + [currently_on] if currently_on > -1 else []
    adj_going   = self.getAdjacentShingleIndexes(self.idx_to_coord(going_to, self.roof_width)) + [going_to]

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

      if (self.foot_down == 0):
        req.scoped_male = iw_bot
        self.mate_suppress_proxy(req)

      elif (self.foot_down == 1):
        req.scoped_male = iw_top
        self.mate_suppress_proxy(req)

  def popShingle(self, shingle_idx, roof_idx):
    shingle = ["inchworm", f"shingle_description_{shingle_idx}", f"shingle_{shingle_idx}"]
    roof = ["inchworm", f"roof_description_0", f"roof_0", f"male_{roof_idx}"]

    # Find all shingle indices adjacent to and including currently_on and going_to.
    adj_current = self.getAdjacentShingleIndexes(self.idx_to_coord(self.on_shingle, self.roof_width)) + [self.on_shingle] if self.on_shingle > -1 else []
    
    req = SuppressMateRequest()
    req.suppress = True

    
    print(f"\tSuppressing shingle {shingle_idx}")

    req.scoped_female = shingle

    req.scoped_male = roof
    self.mate_suppress_proxy(req)

  def pickupShingle(self, shingle_idx, neighbor):
    roof_idx = shingle_idx #will change later, Eli is doing the roof->shingle math

    #go to shingle
    if (self.lastNeighbor != Inchworm.Neighbors.NONE):
      poses = self.mapNeighborToPoses.get(self.lastNeighbor)
      #hover above last place
      self. moveTo(poses[0], 1.0)
      #lift above last place
      self.moveTo(poses[1], 1.0)
      rospy.sleep(1.0)
      #grab the last place
      #lift above the last place
    poses = self.mapNeighborToPoses.get(neighbor)

    #go to lift above next place
    self.moveTo(poses[1], 7.0)
    #hover above next place
    self.moveTo(poses[0], 1.0)
    #plant foot on shingle
    self.moveTo(poses[2], 1.0)
    self.popShingle(self, shingle_idx, roof_idx)
    #move foot up
    self.moveTo(poses[0], 1.0)
    self.moveTo(poses[1], 1.0)

  #THIS IS JUST FOR MOVING AROUND ROOF, NOT PICKING UP SHINGLES
  def move(self, neighbor):
   
    if (self.lastNeighbor != Inchworm.Neighbors.NONE):
      poses = self.mapNeighborToPoses.get(self.lastNeighbor)
      #hover above last place
      self. moveTo(poses[0], 1.0)
      #lift above last place
      self.moveTo(poses[1], 1.0)
      rospy.sleep(1.0)
      #grab the last place
      #lift above the last place

    poses = self.mapNeighborToPoses.get(neighbor)
    #go to lift above next place
    self.moveTo(poses[1], 7.0)
    #hover above next place
    self.moveTo(poses[0], 1.0)
    #plant foot
    self.moveTo(poses[2], 1.0)

    if (neighbor != Inchworm.Neighbors.NONE):
      self.swapFeet()
    
    self.lastNeighbor = self.transform.get(neighbor)


if __name__ == "__main__":
  # This only exists to test the class.

  rospy.init_node("iw_class_test")

  iw = Inchworm(idx=0)
  rospy.sleep(1)

  print(iw.getAdjacentShingleIndexes(iw.idx_to_coord(12, iw.roof_width)))

  print("right")
  iw.move(iw.Neighbors.RIGHT)
  print("upper left")
  iw.move(iw.Neighbors.UPPER_LEFT)
  print("right")
  iw.move(iw.Neighbors.RIGHT)
  print("upper left")
  iw.move(iw.Neighbors.UPPER_LEFT)

  #test things
  print("straight")
  iw.move(Inchworm.Neighbors.NONE)

  #print("grab left")
  #iw.pickupShingle(6, iw.Neighbors.LEFT)
  

 