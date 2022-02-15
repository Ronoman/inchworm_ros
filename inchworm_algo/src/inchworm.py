from enum import Enum
from pickletools import int4

from inchworm_algo.msg import InchwormMsg
import rospy
# all x and y are in array coords currently


class EEStatus(Enum):
    PLANTED = 0
    IN_AIR = 1

class EEShingleStatus(Enum):
    NO_SHINGLE = 0 #should only be with in_air
    INSTALLED = 1 #should only be with planted
    PLACED = 2 #should only be with planted
    ATTACHED = 3 #should only be with in_air

class Behavior(Enum):
    SKELETON = 0


class RobotState(Enum):
    MOVE_TO_DEPOT = 0 
    PICKUP_SHINGLE_FROM_DEPOT = 1
    MOVE_TO_TARGET = 2
    INSTALL_SHINGLE = 3
    PATROL_FRONTIER = 4
    MOVE_SHINGLE = 5
    PROBE_SHINGLE = 6

class Inchworm():
    id = -1
    ee1_position = [-1, -1]
    ee2_position = [-1, -1]
    ee1_status = EEStatus.PLANTED
    ee2_status = EEStatus.PLANTED
    ee1_shingle_stat = EEShingleStatus.INSTALLED
    ee2_shingle_stat = EEShingleStatus.INSTALLED
    behavior = Behavior.SKELETON
    roof = [[]] #occupancy grid


    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

    

    def __init__(self, id = -1, ee1_pos = [-1, -1], ee2_pos = [-1, -1], 
            ee1_stat = EEStatus.PLANTED, ee2_stat = EEStatus.PLANTED, ee1_shingle_stat = EEShingleStatus.NO_SHINGLE,
            ee2_shingle_stat = EEShingleStatus.NO_SHINGLE, behavior = Behavior.SKELETON, width = 1, height = 1, shingle_depot_pos = [0]):
        self.id = id # this value should not change once it is assigned
        self.ee1_position = ee1_pos
        self.ee2_position = ee2_pos
        self.ee1_status = ee1_stat
        self.ee2_status = ee2_stat
        self.ee1_shingle_stat = ee1_shingle_stat
        self.ee2_shingle_stat = ee2_shingle_stat
        self.behavior = behavior
        self.roof_width = width
        self.roof =  [0] * width * height
        self.shingle_depot_pos = shingle_depot_pos
        self.next_tick_time = rospy.Time.now()
        self.robot_state = RobotState.MOVE_TO_DEPOT
        self.target_ee1_pos = ee1_pos
        self.target_ee2_pos = ee2_pos
        self.ee_shingle_neighbor_index = 0
        self.ee_shingle_neighbors = []



    def create_from_message(self, msg):
        self.id = msg.id 
        self.ee1_position = msg.ee1_pos 
        self.ee2_position = msg.ee2_pos 
        self.ee1_status = msg.ee1_status
        self.ee2_status = msg.ee2_status
        self.ee1_shingle_pos = msg.ee1_shingle_pos 
        self.ee2_shingle_pos = msg.ee2_shingle_pos 
        self.behavior = msg.behavior 
        return self

    def place_shingle(self, ee, shingle, roof):
        if (ee == 1):
            shingle.place_shingle(self.ee1_position[0], self.ee1_position[1])
            roof.place_shingle(shingle, self.ee1_position[0], self.ee1_position[1])
            self.roof[self.ee1_position[0] + self.ee1_position[1] * self.roof_width] = 1
            self.ee1_shingle_stat = EEShingleStatus.PLACED
        else:
            shingle.place_shingle(self.ee2_position[0], self.ee2_position[1])
            roof.place_shingle(shingle, self.ee2_position[0], self.ee2_position[1])
            self.roof[self.ee2_position[0] + self.ee2_position[1] * self.roof_width] = 1
            self.ee2_shingle_stat = EEShingleStatus.PLACED
        #do something
        return self

    def release_shingle(self, ee):
        if (ee == 1):
            self.ee1_shingle_stat = EEShingleStatus.NO_SHINGLE
        else:
            self.ee2_shingle_stat = EEShingleStatus.NO_SHINGLE
        return self

    def pickup_shingle(self, ee, shingle, roof):
        shingle.pickup_shingle()
        roof.pickup_shingle(shingle)
        if (ee == 1):
            self.roof[self.ee1_position[0] + self.ee1_position[1] * self.roof_width] = 0
            self.ee1_shingle_stat = EEShingleStatus.ATTACHED
        else:
            self.roof[self.ee2_position[0] + self.ee2_position[1] * self.roof_width] = 0
            self.ee2_shingle_stat = EEShingleStatus.ATTACHED
        #do something
        return self
    
    def install_shingle(self, ee, shingle, roof):
        #figure out x and y -> going off ee1?
        if (ee == 1):
            shingle.install_shingle(self.ee1_position[0], self.ee1_position[1])
            roof.install_shingle(shingle, self.ee1_position[0], self.ee1_position[1])
            self.ee1_shingle_stat = EEShingleStatus.INSTALLED
        else:
            shingle.install_shingle(self.ee2_position[0], self.ee2_position[1])
            roof.install_shingle(shingle, self.ee2_position[0], self.ee2_position[1])
            self.ee2_shingle_stat = EEShingleStatus.INSTALLED
        
        return self

    
    def update_shingle(self, ee, shingle):
        if (ee == 1):
            shingle.x_coord = self.ee1_position[0]
            shingle.y_coord = self.ee1_position[1]
            stat = self.ee1_shingle_stat
        else:
            shingle.x_coord = self.ee2_position[0]
            shingle.y_coord = self.ee2_position[1]
            stat = self.ee2_shingle_stat
        
        #translate to shingle status
        if stat > 2:
            stat = 0
        shingle.shingle_status = stat

        return self

    def read_shingle(self, shingle):
        x = shingle.x_coord
        y = shingle.y_coord
        #update the "roof" to include shingle, if the shingle is in a place
        if ((x != -1) and (y != -1)):
            self.roof[x + y * self.roof_width] = shingle.shingle_status
        #update end effector status?
        return self

    def move_e1(self):
        pass


    def move_e2(self):
        pass

    def rebuild_roof(self):
        pass


    def find_frontier(self):
        pass

    def next_to_placed_shingle(self, pos):
        return False

    def placed_shingle_is_valid(self, shingle):
        return False

    def choose_shingle_target(self, placed_shinge_1, placed_shingle_2): # TODO: change this definition
        
        return {"pos":(0, 0)}

    def dist(ee_pos, target):
        return 0

    def run_one_tick(self, shingles, inchworm_occ): # the roof is passed in because the inchworm has to interact with the real roof

        # general idea is that this contitional is run everytime the robot has to make a desision,
        # Above will go the state machine that runs sub-routines 
        if self.behavior == Behavior.SKELETON:
            rospy.loginfo(f"running inchworm {self.id} for one tick")
            if rospy.Time.now() >= self.next_tick_time:
                # read the shingles at the current ee positions
                # I am assuming that everytime this loop is run, both feet will be on the ground and we will want to read both shingles
                # TODO: allow inchworms to read the full data from a shingle and rebuild based off of that
                # TODO: if the data read does not match the inchworms such that the shingles are out of date, update the shingle
                # TODO: add this to the state machine down below
                self.read_shingle(shingles[self.ee1_position[0]][self.ee1_position[1]])
                self.read_shingle(shingles[self.ee2_position[0]][self.ee2_position[1]])


                # rebuild the roof based on constraints
                self.rebuild_roof()
                # get the coords of the current frontier shingles
                frontier_coords = self.find_frontier()

                # check if either ee is on the frontier, if it is not, move towards shingle depot
                if ( not (self.ee1_position not in frontier_coords and self.ee2_position not in frontier_coords) or
                    self.next_to_placed_shingle(self.ee1_position) or self.next_to_placed_shingle(self.ee2_position) or self.robot_state == RobotState.MOVE_SHINGLE_TO_TARGET):
                    
                    # get the placed shingles  TODO: CHANGE THIS NEED TO INCLUDE MULTIPLE PLACED SHINGLES
                    ee1_placed_shingle = None
                    ee2_placed_shingle = None

                    placed_shingle = None

                    
                    self.target = self.choose_shingle_target(ee1_placed_shingle, ee2_placed_shingle) # TODO: change this definition - this is where real behaviors will happen
                    



                    if Inchworm.dist(self.avg_pos, self.target) > Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord]):
                        if Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord]) == 0: # check if placed shingle is in the target position
                            if inchworm_occ[placed_shingle.y_coord][placed_shingle.x_coord] == 0:
                                self.install_shingle_target = placed_shingle
                                self.robot_state = RobotState.INSTALL_SHINGLE
                                inchworm_occ[placed_shingle.y_coord][placed_shingle.x_coord] = 1 # signel intention to move
                            else:
                                self.robot_state = RobotState.MOVE_TO_DEPOT # TODO: figure out something better to do here
                        elif Inchworm.dist(self.ee1_position, self.target) > Inchworm.dist(self.ee2_position, self.target): # figure out which ee needs to move to get closer to target
                            # TODO: not sure about the below movement logic, on ee will always be infront of the shingle, prventing cool handoffs
                            # move ee1 to placed shingle
                            # probe potental new shingle location
                            # pickup shingle - include probing
                            # place shingle besides ee2
                            # swing around to new valid frontier shingle

                            # get ee1_pos shingle neighbors -- all of them
                            # sort based on lowest distance to target
                            # loop through, probing each shingle, if shingle is free and exists, -- signel intent to probe in inchworm_occ 
                            # as soon as one shingle exists, move to that shingle




                            self.ee_to_move_to(self.get_ee_neighbors(self.ee2_position), 'ee1')
                            self.ee_to_move_to.sort(key = lambda x: Inchworm.dist(x, self.target))
                            

                        else: 
                            # move ee2 to placed shingle
                            # probe potental new placed shingle location
                            # pickup shingle - include probing
                            # place shingle besides ee1
                            # swing around to new valid frontier shingle
                            self.ee_to_move_to(self.get_ee_neighbors(self.ee2_position), 'ee1')
                            self.ee_to_move_to.sort(key = lambda x: Inchworm.dist(x, self.target))


                   
                    # check to see if we know the best path exists & if that shingle is free, if so, initaite move there
                    if (self.roof[self.ee_to_move_to[0][1]][self.ee_to_move_to[0][0]] == 1 and
                            inchworm_occ[self.ee_to_move_to[0][1]][self.ee_to_move_to[0][0]] == 0):
                        inchworm_occ[self.ee_to_move_to[0][1]][self.ee_to_move_to[0][0]] = 1
                        self.to_move("ee1", self.ee_to_move_to[0])
                        self.robot_state = RobotState.MOVE_TO_TARGET

                    # otherwise start probing the points
                    self.robot_state = RobotState.PROBE_SHINGLE


                else:
                    # move toward nearest shingle depot or patrol frontier
                    self.robot_state = RobotState.MOVE_TO_DEPOT
                    

                    



    def create_action(self, shingles, inchworm_occ):
        # this is where all movement will happen
        if self.robot_state == RobotState.MOVE_TO_DEPOT:
            # TODO: probe the direction toward the shingle depot, if the space is free set as new target, otherwise move in a random dircetion -- change to be smart collision avoidence

            pass
        elif self.robot_state == RobotState.PICKUP_SHINGLE_FROM_DEPOT:
            pass
        elif self.robot_state == RobotState.MOVE_TO_TARGET:
            pass
        elif self.robot_state == RobotState.INSTALL_SHINGLE:
            pass
        elif self.robot_state == RobotState.PATROL_FRONTIER:
            pass
        elif self.robot_state == RobotState.MOVE_SHINGLE:
            pass
        elif self.robot_state == RobotState.PROBE_SHINGLE:
            
            if self.check_new_ee_location(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index], roof):
                self.robot_state = RobotState.MOVE_TO_TARGET

            self.ee_shingle_neighbor_index += 1
            pass

    def check_new_ee_location():
        pass


    def to_message(self):
        msg = InchwormMsg()
        msg.id = self.id
        msg.ee1_pos = self.ee1_position
        msg.ee2_pos = self.ee2_position
        msg.ee1_status = self.ee1_status.value
        msg.ee2_status = self.ee2_status.value
        msg.ee1_shingle_stat = self.ee1_shingle_stat.value
        msg.ee2_shingle_stat = self.ee2_shingle_stat.value
        msg.behavior = self.behavior.value
        msg.roof = self.roof
        msg.roof_width = self.roof_width
        msg.shingle_depot_pos = self.shingle_depot_pos
        msg.header.stamp = rospy.Time.now()
        return msg
        



    '''
    TODO:
        - write distance function
        - get neighbors of an ee
        - write check_new_ee_location - will need to probe and indicat where it wants to move to 
        - function to choose target - maybe indicate the target on the viz
        - write rebuild roof
        - write find frontier
        - write code to chose target towards the shingle depot
        - write function to actual move the ee to the target
        - write checker to see if you are at the shingle depot/ update shingle depot location
        - get shingle from shingle depot
        - move shingle logic in run_one_tick
        - get placed shingles near the robot - not sure how this will be handeled

    
    
    '''