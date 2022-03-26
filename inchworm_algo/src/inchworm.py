from enum import Enum
from operator import truediv
from pickletools import int4
from turtle import pos

from numpy import real

from inchworm_algo.msg import InchwormMsg
import rospy
import math
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
import hex_converter
# all x and y are in array coords currently


class EEStatus(Enum):
    PLANTED = 0
    IN_AIR = 1


class EEShingleStatus(Enum):
    NO_SHINGLE = 0  # should only be with in_air
    INSTALLED = 1  # should only be with planted
    PLACED = 2  # should only be with planted
    ATTACHED = 3  # should only be with in_air


class EE(Enum):
    BOTTOM_FOOT = 0  
    TOP_FOOT = 1  


class Behavior(Enum):
    SKELETON = 0


class RobotState(Enum):
    PICKUP_SHINGLE_FROM_DEPOT = 0
    MOVE_TO_TARGET = 1
    INSTALL_SHINGLE = 2
    PATROL_FRONTIER = 3
    MOVE_SHINGLE = 4
    PROBE_SHINGLE = 5
    MAKE_DECISION = 6


# TODO: WE ARE CURRENTLY IGNORRING HALF SHINGLES
class Inchworm():
    id = -1
    bottom_foot_position = [-1, -1]
    top_foot_position = [-1, -1]
    
    

    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    DELAY = 0

    def __init__(self, id=-1, bottom_foot_pos=[-1, -1], top_foot_pos=[-1, -1],
                 bottom_foot_stat=EEStatus.PLANTED, top_foot_stat=EEStatus.PLANTED, bottom_foot_shingle_stat=EEShingleStatus.NO_SHINGLE,
                 top_foot_shingle_stat=EEShingleStatus.NO_SHINGLE, behavior=Behavior.SKELETON, width=1, height=1, shingle_depot_pos=[0]):
        self.id = id  # this value should not change once it is assigned
        self.bottom_foot_position = bottom_foot_pos
        self.top_foot_position = top_foot_pos
        self.bottom_foot_status = bottom_foot_stat
        self.top_foot_status = top_foot_stat
        self.bottom_foot_shingle_stat = bottom_foot_shingle_stat
        self.top_foot_shingle_stat = top_foot_shingle_stat
        self.behavior = behavior
        self.roof_width = width
        self.roof = [ShingleStatus.UNINSTALLED] * width * height
        for i in range(width):
            self.roof[i] = ShingleStatus.INSTALLED
        self.shingle_depot_pos = shingle_depot_pos
        self.next_tick_time = rospy.Time.now()
        self.robot_state = RobotState.MAKE_DECISION
        self.target_bottom_foot_pos = bottom_foot_pos
        self.target_top_foot_pos = top_foot_pos
        self.foot_shingle_neighbor_to_move_to = 0
        self.ee_shingle_neighbors = []
        self.installing_status = 0

        self.shingle_order = self.create_shingle_order(width, height)
        self.claimed_pos = set()

    # this is used to create ox-plow order in shingling
    def create_shingle_order(self, width, height):
        shingle_index_order = []
        if height % 2 == 0:
            for i in range(height ):

                if i % 2 == 0:

                    for j in range(width):
                        shingle_index_order.append([j, i])
                else:
                    for j in range(1, width + 1):
                        shingle_index_order.append([width - j, i])
        else:
            for i in range(height ):

                if i % 2 == 0:

                    for j in range(1, width + 1):
                        shingle_index_order.append([width - j, i])
                else:
                    for j in range(width):
                        shingle_index_order.append([j, i])

        return shingle_index_order

    def create_from_message(self, msg):
        self.id = msg.id
        self.bottom_foot_position = msg.bottom_foot_pos
        self.top_foot_position = msg.top_foot_pos
        self.bottom_foot_status = msg.bottom_foot_status
        self.top_foot_status = msg.top_foot_status
        self.bottom_foot_shingle_pos = msg.bottom_foot_shingle_pos
        self.top_foot_position = msg.top_foot_position
        self.behavior = msg.behavior
        return self

    def get_shingle_state(self, x, y):
        return self.roof[x + y * self.roof_width]

    def set_shingle_state(self, x, y, shingle_state):
        self.roof[x + y * self.roof_width] = shingle_state

    def place_shingle(self, ee, shingle, roof):
        if (ee == 1):
            shingle = shingle.place_shingle(
                self.bottom_foot_position[0], self.bottom_foot_position[1])
            roof.place_shingle(
                shingle, self.bottom_foot_position[0], self.bottom_foot_position[1])
            self.set_shingle_state(self.bottom_foot_position[0], self.bottom_foot_position[1], ShingleStatus.PLACED)
            self.bottom_foot_shingle_stat = EEShingleStatus.PLACED
        else:
            shingle = shingle.place_shingle(
                self.top_foot_position[0], self.top_foot_position[1])
            roof.place_shingle(
                shingle, self.top_foot_position[0], self.top_foot_position[1])
            self.set_shingle_state(self.top_foot_position[0], self.top_foot_position[1], ShingleStatus.PLACED)
            self.top_foot_shingle_stat = EEShingleStatus.PLACED
        # do something
        return self



    def pickup_shingle(self, ee, shingle, roof):
        shingle = shingle.pickup_shingle()
        roof.pickup_shingle(shingle)
        if (ee == 1):
            self.set_shingle_state(self.bottom_foot_position[0], self.bottom_foot_position[1], ShingleStatus.UNINSTALLED)
            self.bottom_foot_shingle_stat = EEShingleStatus.ATTACHED
        else:
            self.set_shingle_state(self.top_foot_position[0], self.top_foot_position[1], ShingleStatus.UNINSTALLED)

            self.top_foot_shingle_stat = EEShingleStatus.ATTACHED
        
        self.shingle_to_move = shingle

    def install_shingle(self, ee, shingle, roof):
        # figure out x and y -> going off bottom_foot?
        if (ee == 1):
            shingle = shingle.install_shingle(
                self.bottom_foot_position[0], self.bottom_foot_position[1])
            roof.install_shingle(
                shingle, self.bottom_foot_position[0], self.bottom_foot_position[1])
            self.bottom_foot_shingle_stat = EEShingleStatus.INSTALLED
        else:
            shingle = shingle.install_shingle(
                self.top_foot_position[0], self.top_foot_position[1])
            roof.install_shingle(
                shingle, self.top_foot_position[0], self.top_foot_position[1])
            self.top_foot_shingle_stat = EEShingleStatus.INSTALLED

        return self

    # TODO: LOOK BACK AT THIS AND MAKE SURE IT MAKES SENSE ONCE WE ARE ACTUALY WRITING TO SHINGLES
    def update_shingle(self, ee, shingle):
        if (ee == 1):
            shingle.x_coord = self.bottom_foot_position[0]
            shingle.y_coord = self.bottom_foot_position[1]
            stat = self.bottom_foot_shingle_stat
        else:
            shingle.x_coord = self.top_foot_position[0]
            shingle.y_coord = self.top_foot_position[1]
            stat = self.top_foot_shingle_stat

        # translate to shingle status
        if stat > 2:
            stat = 0
        shingle.shingle_status = stat

        return self

    def read_shingle(self, shingle):
        x = shingle.x_coord
        y = shingle.y_coord
        # update the "roof" to include shingle, if the shingle is in a place
        if ((x != -1) and (y != -1)):
            self.set_shingle_state(x, y, shingle.shingle_state)
        # update end effector status?
        return self

    def move_bottom_foot(self, new_pos):
        self.bottom_foot_status = EEStatus.IN_AIR
        self.bottom_foot_position = new_pos

    def move_top_foot(self, new_pos):
        self.top_foot_status = EEStatus.IN_AIR
        self.top_foot_position = new_pos

    def calc_inchworm_pos(self):
        '''calculates the effective position of the inchworm'''
        if self.target[1] == (self.bottom_foot_position[1] + self.top_foot_position[1]) /2:
            self.inchworm_pos = [(self.bottom_foot_position[0] + self.top_foot_position[0]) / 2,
                                    (self.bottom_foot_position[1] + self.top_foot_position[1]) / 2]
        elif ((self.bottom_foot_position[1] + self.top_foot_position[1]) /2)%1 != 0:
            # special case if the robot is on a diagonal
            bottom_foot_dis_to_target = Inchworm.dist(self.bottom_foot_position, self.target)
            top_foot_dis_to_target = Inchworm.dist(self.top_foot_position, self.target)
            if top_foot_dis_to_target > bottom_foot_dis_to_target:
                 self.inchworm_pos = self.bottom_foot_position
            else:
                self.inchworm_pos = self.top_foot_position
                
        else:
            rospy.loginfo(f"inchworm {self.id} is in else :{((self.bottom_foot_position[1] + self.top_foot_position[1]) /2)%1}")

            self.inchworm_pos = [max(self.bottom_foot_position[0], self.top_foot_position[0]),
                                    max(self.bottom_foot_position[1], self.top_foot_position[1])] 

        pass
    
    def rebuild_roof(self):  # TODO: IF THIS STARTS TO MAKE THINGS SLOW, MAKE IT NOT RECUSIVE
        for i, occ in enumerate(self.roof):
            if occ == 1:
                x = i % self.roof_width
                y = int(i/self.roof_width)
                self.make_children_valid(x, y)
        pass

    def make_children_valid(self, x, y):
        self.set_shingle_state(x, y, ShingleStatus.PLACED)
        if y % 2 == 0:  # handle an even row
            test_x = x + Inchworm.EVEN_ROW_N_LOOKUP[1][0]
            test_y = y + Inchworm.EVEN_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)
            test_x = x + Inchworm.EVEN_ROW_N_LOOKUP[2][0]
            test_y = y + Inchworm.EVEN_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)
        else:
            test_x = x + Inchworm.ODD_ROW_N_LOOKUP[1][0]
            test_y = y + Inchworm.ODD_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)
            test_x = x + Inchworm.ODD_ROW_N_LOOKUP[2][0]
            test_y = y + Inchworm.ODD_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)


    def claim_pos(self, real_roof, position):
        self.claimed_pos.add(tuple(position))
        real_roof.claim_position(position)
  

    def unclaim_pos(self, real_roof, position):
        self.claimed_pos.remove(tuple(position))
        real_roof.unclaim_position(position)
        

    def check_self_claimed(self, position):
        return tuple(position) in self.claimed_pos

    def valid_uninstalled_foot_positions(self, neighbors, foot_to_move, real_roof):
        '''returns all neighbors where the inchworm can and there are no shingles'''

        move_targets = []
        for n in neighbors:
            if self.get_shingle_state(n[0], n[1]) != ShingleStatus.INSTALLED and (real_roof.get_occ_position([n[0], n[1]]) == 0 or self.check_self_claimed([n[0], n[1]])):
                move_targets.append({"pos": n, "foot": foot_to_move})
        return move_targets

    def valid_installed_foot_positions(self, neighbors, foot_to_move, real_roof):
        '''returns all neighbors where the inchworm can move where the shingles are installed'''
        move_targets = []
        for n in neighbors:
            if self.get_shingle_state(n[0], n[1]) == ShingleStatus.INSTALLED and (real_roof.get_occ_position([n[0], n[1]]) == 0 or self.check_self_claimed([n[0], n[1]])):
                move_targets.append({"pos": n, "foot": foot_to_move})
        return move_targets

    def valid_foot_positions(self, neighbors, foot_to_move, real_roof):
        '''returns all neighbors that it can move a foot to'''
        move_targets = []
        for n in neighbors:
            if (real_roof.get_occ_position([n[0], n[1]]) == 0 or self.check_self_claimed([n[0], n[1]])):
                move_targets.append({"pos": n, "foot": foot_to_move})
        return move_targets

    # TODO: MAKE THE FOLLOWING FUNCTION NOT MAGIC - they have to be magic until we are writing to shingles properly
    # TODO: CHANGE THIS NEED TO INCLUDE MULTIPLE PLACED SHINGLES
    def get_best_placed_shingle(self, shingles):
        neighbors = self.get_shingle_neighbors(self.bottom_foot_position, shingles)
        neighbors.extend(self.get_shingle_neighbors(
            self.top_foot_position, shingles))
        neighbors.sort(key=lambda x: Inchworm.dist(x, self.target))
        shingle_to_move = neighbors[0]
        #rospy.loginfo(f"shingles to move {shingle_to_move}")
        return shingles[shingle_to_move[1]][shingle_to_move[0]]
        
    # TODO: MAKE THE FOLLOWING FUNCTION NOT MAGIC
    def next_to_placed_shingle(self, pos, shingles):
        neighbors = self.get_shingle_pos_neighbors(pos)

        for n in neighbors:
            if n[0] < self.roof_width:
                read_shingle = shingles[n[1]][n[0]]
                if read_shingle is not None:
                    if read_shingle.shingle_status == ShingleStatus.PLACED:
                        return True
        return False

    def placed_shingle_is_valid(self, shingle):
        '''checks to make sure a shingle location is valid for install'''
        validity_count = 0
        if shingle.y_coord % 2 == 0:
            rospy.loginfo("shingle is on a even row")
            test_x = shingle.x_coord + Inchworm.EVEN_ROW_N_LOOKUP[1][0]
            test_y = shingle.y_coord + Inchworm.EVEN_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            test_x = shingle.x_coord + Inchworm.EVEN_ROW_N_LOOKUP[2][0]
            test_y = shingle.y_coord + Inchworm.EVEN_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            if shingle.x_coord == self.roof_width - 1:
                validity_count += 1
            
        else:
            test_x = shingle.x_coord + Inchworm.ODD_ROW_N_LOOKUP[1][0]
            test_y = shingle.y_coord + Inchworm.ODD_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            test_x = shingle.x_coord + Inchworm.ODD_ROW_N_LOOKUP[2][0]
            test_y = shingle.y_coord + Inchworm.ODD_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            if shingle.x_coord == 0:
                validity_count += 1

        return validity_count >= 2

    # TODO: this will be more complicated in the future - could encode behaviors in here
    def choose_shingle_target(self):
        '''chooses the target install location'''
        x_coord = -1
        y_coord = -1

        # rospy.loginfo(f"inchworm {self.id} has a shingle order of {self.shingle_order}")
        for coord in self.shingle_order:
            val = self.get_shingle_state(coord[0], coord[1])
            # rospy.loginfo(val)
            if val != ShingleStatus.INSTALLED:
                x_coord = coord[0]
                y_coord = coord[1]
                # rospy.loginfo(
                #     f"inchworm {self.id} setting tile {coord} as target")
                # if y_coord % 2 == 0:
                #     break
                # else:
                #     if x_coord + 1 < self.roof_width and self.roof[i + 1] == 2:
                #         break
                #     elif x_coord == self.roof_width -1:
                #         break
                break

        return [x_coord, y_coord]

    def dist(ee_pos, target_pos):
        return hex_converter.evenr_distance(ee_pos, target_pos)
        # return math.sqrt((ee_pos[0] - target_pos[0])**2 + (ee_pos[1] - target_pos[1])**2)

    def next_to_shingle_depot(self, shingle_depot_location):
        '''checks to see if the inchworm has either foot next to the shingle depot'''
        #rospy.loginfo(f"bottom_foot at {self.bottom_foot_position}")
        #rospy.loginfo(f"top_foot at {self.top_foot_position}")
        if self.bottom_foot_position[0] == 0:
            if self.bottom_foot_position[1] == shingle_depot_location:
                return True
            elif self.bottom_foot_position[1] > shingle_depot_location:
                self.shingle_depot_pos[0] = shingle_depot_location
        if self.top_foot_position[0] == 0:

            if self.top_foot_position[1] == shingle_depot_location:
                return True
            elif self.top_foot_position[1] > shingle_depot_location:
                self.shingle_depot_pos[0] = shingle_depot_location

        return False

    def get_shingle_pos_neighbors(self, foot_pos):
        '''gets all foot neighbors that exist on the roof'''
        neighbor_pos = []
        if foot_pos[1] % 2 == 0:  # even row lookup
            for n in Inchworm.EVEN_ROW_N_LOOKUP:
                new_neighbor_pos = [foot_pos[0] + n[0], foot_pos[1] + n[1]]
                if new_neighbor_pos != self.bottom_foot_position and new_neighbor_pos != self.top_foot_position:
                    if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof)/self.roof_width and new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                        neighbor_pos.append(
                            [new_neighbor_pos[0], new_neighbor_pos[1]])
        else:
            for n in Inchworm.ODD_ROW_N_LOOKUP:
                new_neighbor_pos = [foot_pos[0] + n[0], foot_pos[1] + n[1]]
                if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof)/self.roof_width and new_neighbor_pos != self.bottom_foot_position and new_neighbor_pos != self.top_foot_position:
                    if new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                        neighbor_pos.append(new_neighbor_pos)
        return neighbor_pos

    def make_state_move_to_depot(self, real_roof):
        '''makes the state move to depot'''
        self.target = [0, real_roof.get_shingle_depot_location(False)]
        # rospy.loginfo(f"inchworm {self.id} going to depot")
        # figures out the position to move toward the shingle depot 
        # TODO:this should be include some path planning that works the same as moving toward the target
        if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
        else:

            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
        if len(self.ee_shingle_neighbors) > 0:
            self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
            self.robot_state = RobotState.MOVE_TO_TARGET
        else:
            self.robot_state = RobotState.MAKE_DECISION
        self.moved_to_bottom = False # not used currently

    def get_shingle_neighbors(self, pos, shingles):
        '''gets all of the neighbors of a shingle position where the shingle is placed'''
        neighbors = self.get_shingle_pos_neighbors(pos)
        shingle_neighbors = []
        for n in neighbors:
            if n[1] < len(self.roof)/self.roof_width and n[0] < self.roof_width:
                read_shingle = shingles[n[1]][n[0]]
                if read_shingle is not None:
                    if read_shingle.shingle_status == ShingleStatus.PLACED:
                        shingle_neighbors.append(n)
        return shingle_neighbors

    def make_decision(self, real_roof):
        # rospy.loginfo(f"robot {self.id} has claimed tiles {self.claimed_pos}")
        # rospy.loginfo(f"robot {self.id} is in state {self.robot_state}")

        # general idea is that this contitional is run everytime the robot has to make a desision,

        if self.robot_state == RobotState.MAKE_DECISION:
            # read the shingles at the current foot positions
            # I am assuming that everytime this loop is run, both feet will be on the ground and we will want to read both shingles
            # TODO: allow inchworms to read the full data from a shingle and rebuild based off of that
            #   if the data read does not match the inchworms such that the shingles are out of date, update the shingle
            

            # TODO: change this, will require a working probing state and being able to read shingles
            self.roof = []
            for row in real_roof.shingle_array:
                for element in row:
                    if element is not None:
                        self.roof.append(element.shingle_status)
                    else:
                        self.roof.append(ShingleStatus.UNINSTALLED)
            
            self.claim_pos(real_roof,self.bottom_foot_position)
            self.claim_pos(real_roof,self.top_foot_position)
            # rebuild the roof based on constraints
            self.rebuild_roof()

            installing = False

            # rospy.loginfo(f"inchworm {self.id} bottom_foot is next to placed shingles : {self.next_to_placed_shingle(self.bottom_foot_position, shingles)}")
            # rospy.loginfo(f"inchworm {self.id} top_foot is next to placed shingles : {self.next_to_placed_shingle(self.top_foot_position, shingles)}")


            # check either of the end effectors are next is next to a placed shingle
            if self.next_to_placed_shingle(self.bottom_foot_position, real_roof.shingle_array) or self.next_to_placed_shingle(self.top_foot_position, real_roof.shingle_array):
                rospy.loginfo(f"inchworm {self.id} is next to a placed shingle")
                
                self.target = self.choose_shingle_target()

                # get the placed shingles  
                placed_shingle = self.get_best_placed_shingle(real_roof.shingle_array)


                self.calc_inchworm_pos()
                # rospy.loginfo(
                #     f"inchworm {self.id} set target at {self.target}")
                # rospy.loginfo(f"moving towards {self.target}")

                # rospy.logwarn(
                #     f"inchworm dis to target{Inchworm.dist(self.avg_pos, self.target)}")
                # rospy.logwarn(
                #     f"shingle dis to target {Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target)}")

                # rospy.loginfo(f"inchworm {self.id} distance {Inchworm.dist(self.avg_pos, self.target)}")
                # rospy.loginfo(f"tile distance {Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target)}")


                # check if the average inchworm position is farther away from the target
                if Inchworm.dist(self.inchworm_pos, self.target) > Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target):
                    # when you are in here, the inchworm is moving along installed shingles, or installing a shingle
                    rospy.loginfo(f"inchworm {self.id} is farther away from the target that the placed shingle")
                    # check if placed shingle is in the target position and should be installed
                    if Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target) == 0.0:
                        if real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord]) == 0 or self.check_self_claimed([placed_shingle.x_coord, placed_shingle.y_coord]):
                            self.install_shingle_target = placed_shingle
                            self.robot_state = RobotState.INSTALL_SHINGLE
                            # signal intention to move
                            self.claim_pos(real_roof, [placed_shingle.x_coord, placed_shingle.y_coord]) 
                            rospy.logwarn(
                                f"inchworm {self.id} installing shingle")
                            installing = True
                            self.installing_status = 1
                        else:
                            self.make_state_move_to_depot(real_roof)

                    # otherwise check to see if the bottom foot is farther away, in which case you will move the bottom foot
                    elif Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
                    else:
                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
                    # if the inchworm is not installing a shingle, initate movement of the inchworm to the target
                    if not installing:
                        # checks to make sure that ee_shingle_neighbors has a valid movement option
                        if (len(self.ee_shingle_neighbors) > 0 and 
                            self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.INSTALLED):
                            # rospy.loginfo(f"inchworm {self.id} is claiming {self.ee_shingle_neighbors[0]['pos']} and initate move")
                            self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
                            self.robot_state = RobotState.MOVE_TO_TARGET
                else:  # The inchworm needs to move a shingle
                    # move a shingle closer to the target
                    rospy.loginfo(f"inchworm {self.id} wants to move a shingle closer to the target")
                    self.shingle_to_move = placed_shingle
                    placed_shingle_location = [
                        placed_shingle.x_coord, placed_shingle.y_coord]
                    # this gets all the neighbors for each foot, and sorts them based on distance to the target
                    top_foot_shingle_neighbors = self.valid_uninstalled_foot_positions(
                        self.get_shingle_pos_neighbors(self.top_foot_position), EE.BOTTOM_FOOT, real_roof)
                    bottom_foot_shingle_neighbors = self.valid_uninstalled_foot_positions(
                        self.get_shingle_pos_neighbors(self.bottom_foot_position), EE.TOP_FOOT, real_roof)
                    top_foot_shingle_neighbors.sort(
                        key=lambda x: Inchworm.dist(x["pos"], self.target))
                    bottom_foot_shingle_neighbors.sort(
                        key=lambda x: Inchworm.dist(x["pos"], self.target))
                    
                    # rospy.loginfo(
                    #     f"placed shingle location {placed_shingle_location}")
                    # rospy.loginfo(
                    #     f"inchworm {self.id} bottom_foot dist to placed shingle :{Inchworm.dist(placed_shingle_location, self.bottom_foot_position)}")
                    # rospy.loginfo(
                    #     f"bottom_foot shingle neighbors: {bottom_foot_shingle_neighbors}")
                    # rospy.loginfo(
                    #     f"inchworm {self.id} top_foot dist to placed shingle :{Inchworm.dist(placed_shingle_location, self.top_foot_position)}")
                    # rospy.loginfo(
                    #     f"top_foot shingle neighbors: {top_foot_shingle_neighbors}")
                    # strip the neighbor options from the dicts to more easly compair them all
                    top_foot_neighbors = [
                        value for elem in top_foot_shingle_neighbors for value in elem.values()]
                    bottom_foot_neighbors = [
                        value for elem in bottom_foot_shingle_neighbors for value in elem.values()]
                    claimed_new_postion = False
                    # if the placed shingle is in one neighbor list and not the other, move the foot that does not have the shingle in the list
                    if placed_shingle_location in top_foot_neighbors and placed_shingle_location not in bottom_foot_neighbors:
                        # move the bottom foot to the placed shingle
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                        self.claim_pos(real_roof, top_foot_shingle_neighbors[0]["pos"])
                        claimed_new_postion = True
                        self.old_bottom_foot = self.bottom_foot_position
                    elif placed_shingle_location in bottom_foot_neighbors and placed_shingle_location not in top_foot_neighbors:
                        # move the top foot to the placed shingle
                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                        self.claim_pos(real_roof, bottom_foot_shingle_neighbors[0]["pos"])
                        claimed_new_postion = True
                        self.old_top_foot = self.top_foot_position
                            

                    # for additional checks, make sure that both shingle lists have at least one entry
                    elif len(bottom_foot_shingle_neighbors) > 0 and len(top_foot_shingle_neighbors) > 0:
                        # if bottom_foot is farther away, move bottom_foot
                        if Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) > Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target):
                            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                            self.claim_pos(real_roof,top_foot_shingle_neighbors[0]["pos"])
                            claimed_new_postion = True
                            self.old_bottom_foot = self.bottom_foot_position
                        # if top_foot is farther away, move top_foot
                        elif Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) < Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target):
                            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                            self.claim_pos(real_roof, bottom_foot_shingle_neighbors[0]["pos"])
                            claimed_new_postion = True
                            self.old_top_foot = self.top_foot_position
                    else:
                        rospy.logwarn(
                            f"inchworm {self.id} could not figure out which end effector to use to move the shingle at {placed_shingle_location}")

                    self.move_shingle_step = 1
                    # rospy.loginfo(f"placed shingle occ grid {real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord])}")
                    # rospy.loginfo(f"robot {self.id} clamed new posiiton {claimed_new_postion}")
                    # try and claim the placed shingle if the inchworm is trying to move a shingle
                    if claimed_new_postion and real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord]) == 0:
                        rospy.logwarn(f"inchworm {self.id} is trying to claim the placed shingle")
                        self.claim_pos(real_roof,[placed_shingle.x_coord, placed_shingle.y_coord])
                        self.robot_state = RobotState.MOVE_SHINGLE
                        # rospy.loginfo(f"inchworm {self.id} moving shingle")
                    else:
                        # otherwise try and move away, this will sometimes throw an exception due to unclaiming, if it does the inchworm should not move
                        try:
                            self.target = [self.target[0], self.target[1] + 1] # TODO: this should be temporary, once we have actual path planning it should not be an issue
                            self.unclaim_pos(real_roof, [placed_shingle.x_coord, placed_shingle.y_coord])
                            if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                                self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
                            else:
                                self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
                            # if the inchworm is not installing a shingle, initate movement of the inchworm to the target
                            if (len(self.ee_shingle_neighbors) > 0 and 
                                self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.INSTALLED and 
                                real_roof.get_occ_position(self.ee_shingle_neighbors[0]["pos"]) == 0
                                ):
                                # rospy.loginfo(f"inchworm {self.id} is claiming {self.ee_shingle_neighbors[0]['pos']} and initate move")
                                self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
                                self.robot_state = RobotState.MOVE_TO_TARGET
                        except Exception as e:
                            rospy.logwarn(f"inchworm {self.id} encountered an exception {e}")

            else:
                # if the inchworm is not next to the shingle depot, move toward it, otherwise pick up a new shingle
                if not self.next_to_shingle_depot(real_roof.get_shingle_depot_location(False)):
                    self.make_state_move_to_depot(real_roof)
                else:
                    self.robot_state = RobotState.PICKUP_SHINGLE_FROM_DEPOT

        return real_roof

    def run_action(self, real_roof):
        # this is where all movement will happen

        if self.robot_state == RobotState.PICKUP_SHINGLE_FROM_DEPOT:
            # place holder for now, shingle depot just spawns a new shingle in the persumed location

            rospy.sleep(Inchworm.DELAY)
            # rospy.loginfo(f"shingle depot at {real_roof.get_shingle_depot_location(False)}")
            # check if the shingle depot has placed a shingle in the new spot
            if real_roof.spawn_shingle(): # TODO: depot currently spawns the shingle in the new location, this will need to change
                self.robot_state = RobotState.MAKE_DECISION
            
        elif self.robot_state == RobotState.MOVE_TO_TARGET:
            # look the foot shingle neighbors to determain which end effector is being moved to target
            if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                # check if foot is not at the target, if so move the foot
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                    self.old_bottom_foot = self.bottom_foot_position
                    self.move_bottom_foot(
                        self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                else:
                    # otherwise plant the foot and unclaim the old position
                    if self.bottom_foot_position != self.old_bottom_foot:
                        self.unclaim_pos(real_roof, self.old_bottom_foot)
                        self.old_bottom_foot = self.bottom_foot_position
                    self.bottom_foot_status = EEStatus.PLANTED
                    self.robot_state = RobotState.MAKE_DECISION
            else:
                # check if foot is not at the target, if so move the foot
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                    self.old_top_foot = self.top_foot_position
                    self.move_top_foot(
                        self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                else:
                    # otherwise plant the foot and unclaim the old position
                    if self.top_foot_position != self.old_top_foot:
                        self.unclaim_pos(real_roof, self.old_top_foot)
                        self.old_top_foot = self.top_foot_position
                    self.top_foot_status = EEStatus.PLANTED
                    self.robot_state = RobotState.MAKE_DECISION

        elif self.robot_state == RobotState.INSTALL_SHINGLE:
            rospy.logwarn(self.placed_shingle_is_valid(self.install_shingle_target))
            if self.placed_shingle_is_valid(self.install_shingle_target):
                # double check that the placed shingle is a valid install 
                # rospy.loginfo(
                #     f"inchworm {self.id} is installing a shingle at {self.target}")
                # installation is a two step process
                # small state machine to allow for multi step install
                if self.installing_status == 1:
                    # figure out which foot needs to install the shingle, this needs to be done once per install
                    if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_foot_positions, real_roof)
                        self.installing_status = 2
                    else:
                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_foot_positions, real_roof)
                        self.installing_status = 2
                # install the shingle and place the foot on the new shingle
                elif self.installing_status == 2: 
                    # determains which foot we are controling 
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                        # insure that the foot is in the correct location, if not we move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                            self.old_bottom_foot = self.bottom_foot_position
                            self.move_bottom_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # install the shingle and place the foot on the newly installed shingle
                            real_roof.install_shingle(self.install_shingle_target)
                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.INSTALLED)
                            if self.bottom_foot_position != self.old_bottom_foot:
                                self.unclaim_pos(real_roof, self.old_bottom_foot)
                                self.old_bottom_foot = self.bottom_foot_position
                            rospy.sleep(Inchworm.DELAY)
                            # rospy.loginfo(
                            #     f"inchworm {self.id} has installed a shingle at {[[self.install_shingle_target.y_coord],[self.install_shingle_target.x_coord]]}")
                            self.bottom_foot_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                            self.installing_status = 0

                    else:
                        # insure that the foot is in the correct location, if not we move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                            self.old_top_foot = self.top_foot_position
                            self.move_top_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # install the shingle and place the foot on the newly installed shingle
                            real_roof.install_shingle(self.install_shingle_target)
                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.INSTALLED)
                            if self.top_foot_position != self.old_top_foot:
                                self.unclaim_pos(real_roof, self.old_top_foot)
                                self.old_top_foot = self.top_foot_position
                            rospy.sleep(Inchworm.DELAY)
                            # rospy.loginfo(
                            #     f"inchworm {self.id} has installed a shingle at {[[self.install_shingle_target.y_coord],[self.install_shingle_target.x_coord]]}")
                            self.top_foot_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                            self.installing_status = 0

        elif self.robot_state == RobotState.PATROL_FRONTIER:
            # TODO: This is where we could implement moving along the frontier, just moving about
            self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_SHINGLE:
            rospy.sleep(Inchworm.DELAY)
            # moving shingles is a multi step process so it requires a state machine in order to be non-blocking
            if self.move_shingle_step == 1:
                self.old_shingle_pos = [
                    self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]

                # check to see which foot you need to move
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    # move the foot to the new position and pick up a shingle
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.bottom_foot_position:
                        rospy.loginfo(
                            f"inchworm {self.id} moving bottom_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_bottom_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])

                        self.shingle_to_move = real_roof.pickup_shingle([self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        self.move_shingle_step = 2

                else:
                    # move the foot to the new position and pick up a shingle
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.top_foot_position:
                        rospy.loginfo(
                            f"inchworm {self.id} moving top_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_top_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        self.shingle_to_move = real_roof.pickup_shingle([self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        self.move_shingle_step = 2

            elif self.move_shingle_step == 2:
                # move the foot to the new location
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                        self.move_bottom_foot(
                            self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    else:
                        # place the shingle
                        self.shingle_to_move = self.shingle_to_move.place_shingle(
                            self.bottom_foot_position[0], self.bottom_foot_position[1])

                        real_roof.place_shingle(self.shingle_to_move, self.bottom_foot_position)
                        self.move_shingle_step = 3
                # moves the foot to the new location
                else:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                        self.move_top_foot(
                            self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    else:
                        # place the shingle
                        self.shingle_to_move = self.shingle_to_move.place_shingle(
                            self.top_foot_position[0], self.top_foot_position[1])
                        real_roof.place_shingle(self.shingle_to_move, self.top_foot_position)

                        self.move_shingle_step = 3

            else:
                # moves the foot to it's old postion
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    self.move_bottom_foot(self.old_bottom_foot)
                    self.bottom_foot_status = EEStatus.PLANTED

                else:
                    self.move_top_foot(self.old_top_foot)
                    self.top_foot_status = EEStatus.PLANTED
                # un claim the positions that it was using
                self.unclaim_pos(real_roof, [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                self.unclaim_pos(real_roof,self.old_shingle_pos)
                # rospy.loginfo(f"shingles claimed after unclaiming {self.claimed_pos}")
                self.robot_state = RobotState.MAKE_DECISION
                self.move_shingle_step = 0

        elif self.robot_state == RobotState.PROBE_SHINGLE:
            # TODO: should move the inchworm to the suposed location and probe it
            
            rospy.sleep(Inchworm.DELAY)
        

        return real_roof

    # returns true if there is a shingle at that location
    def probe(self, location, inchworm_occ, shingles):
        pass

    def decide_on_movement_to_shingle(self, ee_to_move, neighbor_funtion, real_roof):
        '''decides on how to move a given foot to a shingle
            right now this uses greedy movement and in the future we could have it look a few shingles ahead to determain a better path
        '''
        if ee_to_move == EE.BOTTOM_FOOT:
            self.ee_shingle_neighbors = neighbor_funtion(
                self.get_shingle_pos_neighbors(self.top_foot_position), EE.BOTTOM_FOOT, real_roof)
            self.ee_shingle_neighbors.sort(
                key=lambda x: Inchworm.dist(x["pos"], self.target))
            self.old_bottom_foot = self.bottom_foot_position
        else:
            self.ee_shingle_neighbors = neighbor_funtion(
                self.get_shingle_pos_neighbors(self.bottom_foot_position), EE.TOP_FOOT, real_roof)
            self.ee_shingle_neighbors.sort(
                key=lambda x: Inchworm.dist(x["pos"], self.target))
            self.old_top_foot = self.top_foot_position
        self.foot_shingle_neighbor_to_move_to = 0



    def to_message(self):
        msg = InchwormMsg()
        msg.id = self.id
        msg.bottom_foot_pos = self.bottom_foot_position
        msg.top_foot_pos = self.top_foot_position
        msg.bottom_foot_status = self.bottom_foot_status.value
        msg.top_foot_status = self.top_foot_status.value
        msg.bottom_foot_shingle_stat = self.bottom_foot_shingle_stat.value
        msg.top_foot_shingle_stat = self.top_foot_shingle_stat.value
        msg.behavior = self.behavior.value
        if self.bottom_foot_status == EEStatus.PLANTED:
            msg.bottom_foot_valid_neighbors = sum(
                self.get_shingle_pos_neighbors(self.bottom_foot_position), [])
        else:
            msg.bottom_foot_valid_neighbors = []

        if self.top_foot_status == EEStatus.PLANTED:
            msg.top_foot_valid_neighbors = sum(
                self.get_shingle_pos_neighbors(self.top_foot_position), [])
        else:
            msg.top_foot_valid_neighbors = []

        # msg.roof = self.roof
        msg.roof_width = self.roof_width
        msg.shingle_depot_pos = self.shingle_depot_pos
        msg.header.stamp = rospy.Time.now()
        return msg

    '''
    TODO:
        - get rid of all magic functions
            - this will require fixing reading shingles & storage of local roof copies
            - will also require a working probe roof subtask && making sure that when a inchworm installs a shingle it rewrites all neighbors
        - read and write info to roof
        - implement the hex coords
        - create algo that gets the current frontier - involves getting all valid shingle locations based on current roof
        - possible fix for the issue when running 3 worms and 2 moves weirdly
            - actual path planning towards target
            - avoid problem by having an inchworm move first to the frontier and then to the shingle depot
        - create some viz for target

        - path planning so that the inchworm does not move in a greedy fasion, this should treat all other inchworms as obsticals but will only need to run one tick at a time
        - make a new function for choosing the target tile based on the list of valid shingle targets - this is how we can create different patterns


        sudo apt-get install python-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev
        pip install pygame==1.9.6



        functions to change in refactor
            - next_to_placed_shingle
            - get_best_placed_shingle
            - create coord class to use in the enter sim

    
    
    '''
