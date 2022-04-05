#!/usr/bin/env python3

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

DEBUG = False

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
    MAKE_DECISION = 5


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
        self.probe_step = 0
        self.shingle_order = self.create_shingle_order(width, height)
        self.claimed_pos = set()
        self.target = None

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

    def create_diagonal_order(self, width, height):
        in_bounds = lambda x,y: x >= 0 and x < width and y >= 0 and y < height
        
        shingle_index_order = []

        UP_RIGHT_EVEN = [1, 1]
        UP_RIGHT_ODD  = [0, 1]

        # Iterate right to left
        for col in range(width-1, -1, -1):
            start = [col, 1]
            shingle_index_order.append(start)

            next = start
            for row in range(1, height):
                if row % 2 == 0:
                    # even
                    next = [next[0] + UP_RIGHT_EVEN[0], next[1] + UP_RIGHT_EVEN[1]]
                else:
                    # odd
                    next = [next[0] + UP_RIGHT_ODD[0], next[1] + UP_RIGHT_ODD[1]]

                if in_bounds(next[0], next[1]):
                    shingle_index_order.append(next)
        print(shingle_index_order)

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
        if (ee == EE.BOTTOM_FOOT):
            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.UNINSTALLED)

            roof.place_shingle(shingle, self.bottom_foot_position)
            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.PLACED)
            self.bottom_foot_shingle_stat = EEShingleStatus.PLACED
            
            # updates newly placed shingle
            self.update_shingle_with_current_roof(shingle)
            # updates the shingle that the inchworm is plated on
            base_shingle = roof.get_shingle(self.top_foot_position[0], self.top_foot_position[1])
            self.update_shingle_with_current_roof(base_shingle)
        else:
            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.UNINSTALLED)

            roof.place_shingle(shingle, self.top_foot_position)
            self.set_shingle_state(self.top_foot_position[0], self.top_foot_position[1], ShingleStatus.PLACED)
            self.top_foot_shingle_stat = EEShingleStatus.PLACED
            # updates newly placed shingle
            self.update_shingle_with_current_roof(shingle)
            # updates the shingle that the inchworm is plated on
            base_shingle = roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
            self.update_shingle_with_current_roof(base_shingle)




    def pickup_shingle(self, ee, shingle, roof):
        shingle = roof.pickup_shingle([shingle.x_coord, shingle.y_coord])
        if (ee == EE.BOTTOM_FOOT):
            self.set_shingle_state(self.bottom_foot_position[0], self.bottom_foot_position[1], ShingleStatus.UNINSTALLED)
            self.bottom_foot_shingle_stat = EEShingleStatus.ATTACHED
            
            # updates the shingle that the inchworm is plated on
            base_shingle = roof.get_shingle(self.top_foot_position[0], self.top_foot_position[1])
            self.update_shingle_with_current_roof(base_shingle)
        else:
            self.set_shingle_state(self.top_foot_position[0], self.top_foot_position[1], ShingleStatus.UNINSTALLED)
            self.top_foot_shingle_stat = EEShingleStatus.ATTACHED

            # updates the shingle that the inchworm is plated on
            base_shingle = roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
            self.update_shingle_with_current_roof(base_shingle)

        
        self.shingle_to_move = shingle




    def install_shingle(self, ee, shingle, roof):
        # figure out x and y -> going off bottom_foot?
        if (ee == EE.BOTTOM_FOOT):
            roof.install_shingle(shingle)
            self.bottom_foot_shingle_stat = EEShingleStatus.INSTALLED
            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.INSTALLED)
            
            # updates newly installed shingle
            self.update_shingle_with_current_roof(shingle)
            # updates the shingle that the inchworm is plated on
            base_shingle = roof.get_shingle(self.top_foot_position[0], self.top_foot_position[1])
            self.update_shingle_with_current_roof(base_shingle)


        else:
            roof.install_shingle(shingle)
            self.top_foot_shingle_stat = EEShingleStatus.INSTALLED

            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.INSTALLED)

            # updates newly installed shingle
            self.update_shingle_with_current_roof(shingle)
            # updates the shingle that the inchworm is plated on
            base_shingle = roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
            self.update_shingle_with_current_roof(base_shingle)


    

    def read_shingle_at(self, real_roof, shingle_coord):
        shingle = real_roof.get_shingle(shingle_coord[0], shingle_coord[1])

        # if shingle is not None:

        # update the "roof" to include shingle, if the shingle is in a place
        self.set_shingle_state(shingle_coord[0], shingle_coord[1], shingle.shingle_status) 
        neighbor_status = shingle.get_neighbor_locations_and_status()
        i = 0
        for key, value in neighbor_status.items():
            
            # rospy.loginfo(f"inchworm {self.id} updating {key} with {value} on shingle {shingle_coord} using {key}, {neighbor_status}")
            # rospy.logwarn(self.roof)
            if (key[0] > -1 and key[0] < self.roof_width) and (key[1] > -1 and key[1] < len(self.roof)/self.roof_width):
                    if value == ShingleStatus.INSTALLED:
                        self.set_shingle_state(key[0], key[1], ShingleStatus.INSTALLED)
                    elif value == ShingleStatus.PLACED:
                        if self.get_shingle_state(key[0], key[1]) == ShingleStatus.INSTALLED:
                            new_shingle_status = ShingleStatus.INSTALLED
                        else:
                            new_shingle_status = ShingleStatus.PLACED
                        neighbor_index = shingle.convert_to_neighbor_index(key)
                        try:
                            shingle.update_neighbor(neighbor_index, new_shingle_status)
                        except Exception:
                            pass
        # else:
        #     self.set_shingle_state(shingle_coord[0], shingle_coord[1], ShingleStatus.UNINSTALLED)
        #     rospy.logwarn(f"inchworm {self.id} has a plated foot on an empty position")
            





    def update_shingle_with_current_roof(self, shingle):
        if shingle.y_coord % 2 == 0:
            realative_neighbors = self.EVEN_ROW_N_LOOKUP
        else:
            realative_neighbors = self.ODD_ROW_N_LOOKUP
        for i, v in enumerate(realative_neighbors):
            neighbor_coord = [shingle.x_coord + v[0], shingle.y_coord + v[1]]
            if (neighbor_coord[0] > -1 and neighbor_coord[0] < self.roof_width) and (neighbor_coord[1] > -1 and neighbor_coord[1] < len(self.roof)/self.roof_width):
                
                shingle.update_neighbor(i, self.get_shingle_state(neighbor_coord[0], neighbor_coord[1]))

        


    def move_bottom_foot(self, new_pos):
        self.bottom_foot_status = EEStatus.IN_AIR
        self.bottom_foot_position = new_pos

    def move_top_foot(self, new_pos):
        self.top_foot_status = EEStatus.IN_AIR
        self.top_foot_position = new_pos

    def calc_inchworm_pos(self):
        '''calculates the effective position of the inchworm'''
        if self.target[1] == (self.bottom_foot_position[1] + self.top_foot_position[1]) /2:
            inchworm_pos = [(self.bottom_foot_position[0] + self.top_foot_position[0]) / 2,
                                    (self.bottom_foot_position[1] + self.top_foot_position[1]) / 2]
        elif ((self.bottom_foot_position[1] + self.top_foot_position[1]) /2)%1 != 0:
            # special case if the robot is on a diagonal
            bottom_foot_dis_to_target = Inchworm.dist(self.bottom_foot_position, self.target)
            top_foot_dis_to_target = Inchworm.dist(self.top_foot_position, self.target)
            if top_foot_dis_to_target > bottom_foot_dis_to_target:
                 inchworm_pos = self.bottom_foot_position
            else:
                inchworm_pos = self.top_foot_position
                
        else:
            inchworm_pos = [max(self.bottom_foot_position[0], self.top_foot_position[0]),
                                    max(self.bottom_foot_position[1], self.top_foot_position[1])]

        return inchworm_pos
    
    def rebuild_roof(self):  # TODO: IF THIS STARTS TO MAKE THINGS SLOW, MAKE IT NOT RECUSIVE
        for i, occ in enumerate(self.roof):
            if occ == 1:
                x = i % self.roof_width
                y = int(i/self.roof_width)
                self.make_children_valid(x, y)

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
            # self.roof = []
            # for row in real_roof.shingle_array:
            #     for element in row:
            #         if element is not None:
            #             self.roof.append(element.shingle_status)
            #         else:
            #             self.roof.append(ShingleStatus.UNINSTALLED)

            self.read_shingle_at(real_roof, self.bottom_foot_position)
            self.read_shingle_at(real_roof, self.top_foot_position)
            
            self.claim_pos(real_roof,self.bottom_foot_position)
            self.claim_pos(real_roof,self.top_foot_position)
            # rebuild the roof based on constraints
            # self.rebuild_roof()

            installing = False

            # rospy.loginfo(f"inchworm {self.id} bottom_foot is next to placed shingles : {self.next_to_placed_shingle(self.bottom_foot_position, shingles)}")
            # rospy.loginfo(f"inchworm {self.id} top_foot is next to placed shingles : {self.next_to_placed_shingle(self.top_foot_position, shingles)}")


            # check either of the end effectors are next is next to a placed shingle
            if self.next_to_placed_shingle(self.bottom_foot_position, real_roof.shingle_array) or self.next_to_placed_shingle(self.top_foot_position, real_roof.shingle_array):
                rospy.loginfo(f"inchworm {self.id} is next to a placed shingle")
                
                self.target = self.choose_shingle_target()

                # get the placed shingles  
                placed_shingle = self.get_best_placed_shingle(real_roof.shingle_array)


                inchworm_pos = self.calc_inchworm_pos()
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
                if Inchworm.dist(inchworm_pos, self.target) > Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target):
                    # when you are in here, the inchworm is moving along installed shingles, or installing a shingle
                    rospy.loginfo(f"inchworm {self.id} is farther away from the target that the placed shingle")
                    # check if placed shingle is in the target position and should be installed
                    if Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target) == 0.0:
                        if real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord]) == 0 or self.check_self_claimed([placed_shingle.x_coord, placed_shingle.y_coord]):
                            self.install_shingle_target = placed_shingle
                            self.robot_state = RobotState.INSTALL_SHINGLE
                            # signal intention to move
                            self.claim_pos(real_roof, [placed_shingle.x_coord, placed_shingle.y_coord]) 
                            # rospy.logwarn(
                            #     f"inchworm {self.id} installing shingle")
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
                        if Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) > Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target) and top_foot_shingle_neighbors[0]["pos"] != placed_shingle_location:
                            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                            self.claim_pos(real_roof,top_foot_shingle_neighbors[0]["pos"])
                            rospy.loginfo(f"inchworm {self.id} wants to move it's bottom foot")

                            claimed_new_postion = True
                            self.old_bottom_foot = self.bottom_foot_position
                        # if top_foot is farther away, move top_foot
                        elif Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) < Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target) and bottom_foot_shingle_neighbors[0]["pos"] != placed_shingle_location:
                            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                            self.claim_pos(real_roof, bottom_foot_shingle_neighbors[0]["pos"])
                            rospy.loginfo(f"inchworm {self.id} wants to move it's top foot")
                            claimed_new_postion = True
                            self.old_top_foot = self.top_foot_position
                        else:
                            rospy.logwarn(f"inchworm {self.id} encoutered an error")
                    else:
                        rospy.logwarn(
                            f"inchworm {self.id} could not figure out which end effector to use to move the shingle at {placed_shingle_location}")

                    self.move_shingle_step = 1
                    self.probe_step = 0
                    self.original_bottom_foot_pos = self.bottom_foot_position
                    self.original_top_foot_pos = self.top_foot_position
                    # rospy.loginfo(f"placed shingle occ grid {real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord])}")
                    # rospy.loginfo(f"robot {self.id} clamed new posiiton {claimed_new_postion}")
                    # try and claim the placed shingle if the inchworm is trying to move a shingle
                    if claimed_new_postion and real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord]) == 0:
                        self.claim_pos(real_roof,[placed_shingle.x_coord, placed_shingle.y_coord])
                        self.robot_state = RobotState.MOVE_SHINGLE
                        # rospy.loginfo(f"inchworm {self.id} moving shingle")
                    else:
                        # otherwise try and move away, this will sometimes throw an exception due to unclaiming, if it does the inchworm should not move
                        try:
                            rospy.logwarn(f"inchworm {self.id} could not claim both shingles")
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
                    rospy.loginfo(f"inchworm {self.id} moving towards depot")
                    self.make_state_move_to_depot(real_roof)
                else:
                    rospy.loginfo(f"inchworm {self.id} is picking up a new shingle from the depot")
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
            elif self.next_to_placed_shingle(self.bottom_foot_position, real_roof.shingle_array) or self.next_to_placed_shingle(self.top_foot_position, real_roof.shingle_array):
                rospy.loginfo(f"inchworm {self.id} failed to spawn shingle, but is next to a spawned shingle")
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
            if self.placed_shingle_is_valid(self.install_shingle_target):
                # double check that the placed shingle is a valid install 
                # rospy.loginfo(
                #     f"inchworm {self.id} is installing a shingle at {self.target}")
                # installation is a two step process
                # small state machine to allow for multi step install
                '''
                create pre step where we move to the target, and check if there is a shingle there
                    - set original foot positions
                    - probe the potental point
                    - if 
                '''

                if self.installing_status == 1:

                    # figure out which foot needs to install the shingle, this needs to be done once per install
                    if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_foot_positions, real_roof)
                        self.installing_status = 2
                    else:
                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_foot_positions, real_roof)
                        self.installing_status = 2

                elif self.installing_status == 2:
                    self.original_bottom_foot_pos = self.bottom_foot_position
                    self.original_top_foot_pos = self.top_foot_position
                    status = self.probe(real_roof, [self.install_shingle_target.x_coord, self.install_shingle_target.y_coord])
                    if status[0] == 0:
                        if status[1]:
                            rospy.loginfo(f"inchworm {self.id} found an shingle install location at {[self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]}")
                            self.installing_status = 2
                        else:
                            rospy.loginfo(f"inchworm {self.id} did not find an shingle at {[self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]}")
                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.UNINSTALLED)
                            if self.bottom_foot_position[0] == self.install_shingle_target.x_coord and self.bottom_foot_position[1] == self.install_shingle_target.y_coord:
                                base_shingle = real_roof.get_shingle(self.top_foot_position[0], self.top_foot_position[1])
                                self.update_shingle_with_current_roof(base_shingle)
                            else:
                                base_shingle = real_roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
                                self.update_shingle_with_current_roof(base_shingle)
                            self.installing_status = 6

                    
                    self.installing_status = 3
                # install the shingle and place the foot on the new shingle
                elif self.installing_status == 3: 
                    # determains which foot we are controling 
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                        # insure that the foot is in the correct location, if not we move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                            self.old_bottom_foot = self.bottom_foot_position
                            self.move_bottom_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # install the shingle and place the foot on the newly installed shingle
                            self.install_shingle(EE.BOTTOM_FOOT, self.install_shingle_target, real_roof)               
                            self.installing_status = 4

                    else:
                        # insure that the foot is in the correct location, if not we move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                            self.old_top_foot = self.top_foot_position
                            self.move_top_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # install the shingle and place the foot on the newly installed shingle
                            self.install_shingle(EE.TOP_FOOT, self.install_shingle_target, real_roof)
                            self.installing_status = 4
                elif self.installing_status == 4:
                    if self.original_bottom_foot_pos != self.bottom_foot_position:
                        self.move_bottom_foot(self.original_bottom_foot_pos)
                        self.bottom_foot_status = EEStatus.PLANTED
                    if self.original_top_foot_pos != self.top_foot_position:
                        self.move_top_foot(self.original_top_foot_pos)
                        self.top_foot_status = EEStatus.PLANTED
                    self.unclaim_pos(real_roof, [self.install_shingle_target.x_coord, self.install_shingle_target.y_coord])
                    self.robot_state = RobotState.MAKE_DECISION
                    self.installing_status = 0



        elif self.robot_state == RobotState.PATROL_FRONTIER:
            # TODO: This is where we could implement moving along the frontier, just moving about
            self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_SHINGLE:
            rospy.sleep(Inchworm.DELAY)
            # moving shingles is a multi step process so it requires a state machine in order to be non-blocking
            if self.move_shingle_step == 1:
                    
                status = self.probe(real_roof, [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                if status[0] == 0:
                    if status[1]:
                        rospy.loginfo(f"inchworm {self.id} found an shingle at {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_shingle_step = 2
                    else:
                        rospy.loginfo(f"inchworm {self.id} did not find an shingle at {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.set_shingle_state(self.shingle_to_move.x_coord, self.shingle_to_move.y_coord, ShingleStatus.UNINSTALLED)
                        if self.bottom_foot_position[0] == self.install_shingle_target.x_coord and self.bottom_foot_position[1] == self.shingle_to_move.y_coord:
                                base_shingle = real_roof.get_shingle(self.top_foot_position[0], self.top_foot_position[1])
                                self.update_shingle_with_current_roof(base_shingle)
                        else:
                            base_shingle = real_roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
                            self.update_shingle_with_current_roof(base_shingle)
                        self.move_shingle_step = 6
                    
                pass # check the placed shingle pos
            elif self.move_shingle_step == 2:
                
                status = self.probe(real_roof, self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                if status[0] == 0:
                    if not status[1]:
                        rospy.loginfo(f"inchworm {self.id} did not find an shingle at {self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]['pos']}")

                        self.move_shingle_step = 3
                    else:
                        rospy.loginfo(f"inchworm {self.id} found an shingle at {self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]['pos']}")
                        # create new shingle in this position
                        shingle_target_postion = self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]['pos']
                        self.read_shingle_at(real_roof, shingle_target_postion)
                        if self.bottom_foot_position[0] == shingle_target_postion[0] and self.bottom_foot_position[1] == shingle_target_postion[1]:
                                base_shingle = real_roof.get_shingle(self.top_foot_position[0], self.top_foot_position[1])
                                self.update_shingle_with_current_roof(base_shingle)
                        else:
                            base_shingle = real_roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
                            self.update_shingle_with_current_roof(base_shingle)
                        self.move_shingle_step = 6
                    
            elif self.move_shingle_step == 3:
                self.old_shingle_pos = [
                    self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]

                # check to see which foot you need to move
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    # move the foot to the new position and pick up a shingle
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.bottom_foot_position:
                        # rospy.loginfo(
                        #     f"inchworm {self.id} moving bottom_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_bottom_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        self.pickup_shingle(EE.BOTTOM_FOOT, self.shingle_to_move, real_roof)
                else:
                    # move the foot to the new position and pick up a shingle
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.top_foot_position:
                        # rospy.loginfo(
                        #     f"inchworm {self.id} moving top_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_top_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        self.pickup_shingle(EE.TOP_FOOT, self.shingle_to_move, real_roof)
                        
                self.move_shingle_step = 4
                        

            elif self.move_shingle_step == 4:
                # move the foot to the new location
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                        self.move_bottom_foot(
                            self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    else:
                        self.place_shingle(EE.BOTTOM_FOOT, self.shingle_to_move, real_roof)
                        self.move_shingle_step = 5
                # moves the foot to the new location
                else:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                        self.move_top_foot(
                            self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    else:
                        # place the shingle
                        self.place_shingle(EE.TOP_FOOT, self.shingle_to_move, real_roof)
                        self.move_shingle_step = 5

            else:
                # moves the foot to it's old postion
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    self.move_bottom_foot(self.original_bottom_foot_pos)
                    self.bottom_foot_status = EEStatus.PLANTED

                else:
                    self.move_top_foot(self.original_top_foot_pos)
                    self.top_foot_status = EEStatus.PLANTED
                # un claim the positions that it was using
                try:
                    self.unclaim_pos(real_roof, [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                    self.unclaim_pos(real_roof,self.old_shingle_pos)
                except Exception as e:
                    rospy.logwarn(e)
                    rospy.logwarn(self.move_shingle_step)
                # rospy.loginfo(f"shingles claimed after unclaiming {self.claimed_pos}")
                self.robot_state = RobotState.MAKE_DECISION
                self.move_shingle_step = 0
                self.probe_step = 0

        

        return real_roof

    # returns true if there is a shingle at that location
    def probe(self, real_roof, shingle_probe_pos):
        '''
            - move to shingle to probe if you are not already there
            - gets the status of the shingle from the roof
        '''

        # rospy.loginfo(f"inchworm {self.id} probe shingle step of {self.probe_step}")
        if self.probe_step == 0:
            self.probe_status = False
            self.old_shingle_pos = shingle_probe_pos
            if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                if shingle_probe_pos != self.bottom_foot_position:
                    # rospy.loginfo("moving bottom foot")
                    self.old_bottom_foot = self.bottom_foot_position
                    self.move_bottom_foot(shingle_probe_pos)
                else:
                    self.probe_step = 1
            else:
                rospy.loginfo(f"{shingle_probe_pos}, {self.top_foot_position}")
                if shingle_probe_pos != self.top_foot_position:
                    # rospy.loginfo("moving top foot")
                    self.old_top_foot = self.top_foot_position
                    self.move_top_foot(shingle_probe_pos)
                else:
                    self.probe_step = 1
        elif self.probe_step == 1:
            self.probe_status = (real_roof.get_shingle(shingle_probe_pos[0], shingle_probe_pos[1]) is not None and (real_roof.get_shingle(shingle_probe_pos[0], shingle_probe_pos[1]).shingle_status == ShingleStatus.INSTALLED or
                real_roof.get_shingle(shingle_probe_pos[0], shingle_probe_pos[1]).shingle_status == ShingleStatus.PLACED))
            self.probe_step = 2
        else:
            
            self.probe_step = 0
        if self.probe_step == 2:
            return (0, self.probe_status)
        else:
            return (-1, False)


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
        msg_roof = []
        for shingle in self.roof:
            msg_roof.append(shingle.value)
        msg.roof = msg_roof
        # msg.roof = self.roof
        msg.roof_width = self.roof_width
        msg.shingle_depot_pos = self.shingle_depot_pos
        if self.target is not None:
            msg.target = self.target
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



        

     Things to do to make imperfect info happen
        - read from shingles
        - update roof state based on reads
        - probe before you try and place a shingle
        - update based on probe
        - discovery mode to see if there is a placed shingle
            - make inchworms remeber one is?
            - write to a shingle below it if there is a placed shingle there
        - 






        Dealing with imperfect info
            When moving
                - trust our current world on installed shingles
                - read and update world
                    - especialy if installed neighbors is out of data
            When moving shingle
                - read shingle data
                - if placed shingle data & inchworm wants to move
                    - claim both pos
                    - probe both
                    - if new data - both probe & try and read
                        - update world & shingle data
                            - especialy info on installed shingles
                        - move back to org pos
                        - make a new decision
                    - otherwise
                        - move shingle
                        - update shingle data
            When installing shingle
                - update info on the shingle that you are installing from


            - read shingles
            - store read shingles
            - rebuild roof based on data
            - write to shingles based on world view
            - update shingles to include placed shingle info
    
    '''

if __name__ == "__main__":
    iw = Inchworm(width=10, height=10)
    iw.create_diagonal_order()