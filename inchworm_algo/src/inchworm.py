from enum import Enum
from pickletools import int4

from inchworm_algo.msg import InchwormMsg
import rospy
import math
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
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
    MOVE_TO_DEPOT = 0
    PICKUP_SHINGLE_FROM_DEPOT = 1
    MOVE_TO_TARGET = 2
    INSTALL_SHINGLE = 3
    PATROL_FRONTIER = 4
    MOVE_SHINGLE = 5
    PROBE_SHINGLE = 6
    MAKE_DECISION = 7


# TODO: WE ARE CURRENTLY IGNORRING HALF SHINGLES
class Inchworm():
    id = -1
    bottom_foot_position = [-1, -1]
    top_foot_position = [-1, -1]
    bottom_foot_status = EEStatus.PLANTED
    top_foot_status = EEStatus.PLANTED
    bottom_foot_shingle_stat = EEShingleStatus.INSTALLED
    top_foot_shingle_stat = EEShingleStatus.INSTALLED
    behavior = Behavior.SKELETON
    roof = []  # occupancy grid

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
        self.ee_shingle_neighbor_index = 0
        self.ee_shingle_neighbors = []
        self.installing_status = 0

        self.shingle_order = self.create_shingle_order(width, height)

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
        self.top_foot_shingle_pos = msg.top_foot_shingle_pos
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

    def release_shingle(self, ee):
        if (ee == 1):
            self.bottom_foot_shingle_stat = EEShingleStatus.NO_SHINGLE
        else:
            self.top_foot_shingle_stat = EEShingleStatus.NO_SHINGLE
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
        # do something
        return self

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

    # TODO: LOOK BACK AT THIS AND MAKE SURE IT MAKES SENSE
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
        pass

    # TODO: MAKE THE FOLLOWING TWO FUNCTION NOT MAGIC
    def next_to_placed_shingle(self, pos, shingles):
        neighbors = self.get_ee_neighbors(pos)

        for n in neighbors:
            if n[0] < self.roof_width:
                read_shingle = shingles[n[1]][n[0]]
                if read_shingle is not None:
                    if read_shingle.shingle_status == ShingleStatus.PLACED:
                        return True
        return False

    

    def ee_to_move_to_free_space(self, neighbors, ee_to_move):
        move_targets = []
        for n in neighbors:
            if self.get_shingle_state(n[0], n[1]) != ShingleStatus.INSTALLED:
                move_targets.append({"pos": n, "ee": ee_to_move})
        return move_targets

    def ee_to_move_to_shingle(self, neighbors, ee_to_move):
        move_targets = []
        for n in neighbors:
            if self.get_shingle_state(n[0], n[1]) == ShingleStatus.INSTALLED:
                move_targets.append({"pos": n, "ee": ee_to_move})
        return move_targets

    def ee_to_move_to(self, neighbors, ee_to_move):
        move_targets = []
        for n in neighbors:
            move_targets.append({"pos": n, "ee": ee_to_move})
        return move_targets

    # TODO: MAKE THE FOLLOWING TWO FUNCTION NOT MAGIC
    def get_best_placed_shingle(self, shingles):
        neighbors = self.get_shingle_neighbors(self.bottom_foot_position, shingles)
        neighbors.extend(self.get_shingle_neighbors(
            self.top_foot_position, shingles))
        neighbors.sort(reverse=True, key=lambda x: Inchworm.dist(x, [0, 0]))
        shingle_to_move = neighbors[0]
        return shingles[shingle_to_move[1]][shingle_to_move[0]]

    def placed_shingle_is_valid(self, shingle):
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
            if shingle.x_coord == self.roof_width:
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
    def choose_shingle_target(self, placed_shinge_loc):
        x_coord = -1
        y_coord = -1

        # rospy.loginfo(f"inchworm {self.id} has a shingle order of {self.shingle_order}")
        for coord in self.shingle_order:
            val = self.get_shingle_state(coord[0], coord[1])
            # rospy.loginfo(val)
            if val != ShingleStatus.INSTALLED:
                x_coord = coord[0]
                y_coord = coord[1]
                rospy.loginfo(
                    f"inchworm {self.id} setting tile {coord} as target")
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
        return math.sqrt((ee_pos[0] - target_pos[0])**2 + (ee_pos[1] - target_pos[1])**2)

    def next_to_shingle_depot(self, shingle_depot):

        rospy.loginfo(f"bottom_foot at {self.bottom_foot_position}")
        rospy.loginfo(f"top_foot at {self.top_foot_position}")
        if self.bottom_foot_position[0] == 0:
            if self.bottom_foot_position[1] == shingle_depot.get_location():
                return True
            elif self.bottom_foot_position[1] > shingle_depot.get_location():
                self.shingle_depot_pos[0] = shingle_depot.get_location()
        if self.top_foot_position[0] == 0:

            if self.top_foot_position[1] == shingle_depot.get_location():
                return True
            elif self.top_foot_position[1] > shingle_depot.get_location():
                self.shingle_depot_pos[0] = shingle_depot.get_location()

        return False

    def get_ee_neighbors(self, ee_pos, use_offset=True):
        neighbor_pos = []
        if ee_pos[1] % 2 == 0:  # even row lookup
            for n in Inchworm.EVEN_ROW_N_LOOKUP:
                new_neighbor_pos = [ee_pos[0] + n[0], ee_pos[1] + n[1]]
                if new_neighbor_pos != self.bottom_foot_position and new_neighbor_pos != self.top_foot_position:
                    if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof)/self.roof_width and new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                        neighbor_pos.append(
                            [new_neighbor_pos[0], new_neighbor_pos[1]])
        else:
            for n in Inchworm.ODD_ROW_N_LOOKUP:
                new_neighbor_pos = [ee_pos[0] + n[0], ee_pos[1] + n[1]]
                if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof)/self.roof_width and new_neighbor_pos != self.bottom_foot_position and new_neighbor_pos != self.top_foot_position:
                    if new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                        neighbor_pos.append(new_neighbor_pos)
        return neighbor_pos

    def make_state_move_to_depot(self):
        self.target = [0, self.shingle_depot_pos[0]]
        rospy.loginfo(f"inchworm {self.id} going to depot")
        self.robot_state = RobotState.MOVE_TO_DEPOT
        self.moved_to_bottom = False

    def get_shingle_neighbors(self, pos, shingles):
            neighbors = self.get_ee_neighbors(pos)
            shingle_neighbors = []
            for n in neighbors:
                if n[1] < len(self.roof)/self.roof_width and n[0] < self.roof_width:
                    read_shingle = shingles[n[1]][n[0]]
                    if read_shingle is not None:
                        if read_shingle.shingle_status == ShingleStatus.PLACED:
                            shingle_neighbors.append(n)
            return shingle_neighbors

    def run_one_tick(self, shingles, inchworm_occ, shingle_depots):

        # general idea is that this contitional is run everytime the robot has to make a desision,
        if self.behavior == Behavior.SKELETON:
            rospy.loginfo(
                f"running inchworm {self.id} for one tick {self.robot_state}")
            # currently this is the only action that can take multiple itterations
            if self.robot_state == RobotState.MAKE_DECISION:
                # read the shingles at the current ee positions
                # I am assuming that everytime this loop is run, both feet will be on the ground and we will want to read both shingles
                # TODO: allow inchworms to read the full data from a shingle and rebuild based off of that
                # TODO: if the data read does not match the inchworms such that the shingles are out of date, update the shingle
                

                # TODO: change this, will require a working probing state
                self.roof = []
                for row in shingles:
                    for element in row:
                        if element is not None:
                            self.roof.append(element.shingle_status)
                        else:
                            self.roof.append(ShingleStatus.UNINSTALLED)

                # in theory this block should not be needed
                inchworm_occ[self.bottom_foot_position[1]][self.bottom_foot_position[0]] = 1
                inchworm_occ[self.top_foot_position[1]][self.top_foot_position[0]] = 1

                # rebuild the roof based on constraints
                self.rebuild_roof()

                installing = False

                # this is a hack that can be removed once we have hex coords fully implemented
                if (self.bottom_foot_position[1] % 2 == 0 or self.top_foot_position[1] % 2 == 0):
                    self.avg_pos = [((self.bottom_foot_position[0] + self.top_foot_position[0])/2) +
                                    0.5, (self.bottom_foot_position[1] + self.top_foot_position[1])/2]
                else:
                    self.avg_pos = [(self.bottom_foot_position[0] + self.top_foot_position[0])/2,
                                    (self.bottom_foot_position[1] + self.top_foot_position[1])/2]

                # rospy.loginfo(f"inchworm {self.id} bottom_foot is next to placed shingles : {self.next_to_placed_shingle(self.bottom_foot_position, shingles)}")
                # rospy.loginfo(f"inchworm {self.id} top_foot is next to placed shingles : {self.next_to_placed_shingle(self.top_foot_position, shingles)}")


                # check either of the end effectors are next is next to a placed shingle
                if self.next_to_placed_shingle(self.bottom_foot_position, shingles) or self.next_to_placed_shingle(self.top_foot_position, shingles):

                    # get the placed shingles  TODO: CHANGE THIS NEED TO INCLUDE MULTIPLE PLACED SHINGLES

                    placed_shingle = self.get_best_placed_shingle(shingles)

                    # TODO: change this definition - this is where real behaviors will happen
                    self.target = self.choose_shingle_target(placed_shingle)
                    rospy.loginfo(
                        f"inchworm {self.id} set target at {self.target}")
                    rospy.loginfo(f"moving towards {self.target}")

                    rospy.logwarn(
                        f"pos d to target{Inchworm.dist(self.avg_pos, self.target)}")
                    rospy.logwarn(
                        f"s d to target {Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target)}")


                    # check if the average inchworm position is farther away from the target
                    if Inchworm.dist(self.avg_pos, self.target) > Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target):
                        # check if placed shingle is in the target position
                        if Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target) == 0.0:
                            if inchworm_occ[placed_shingle.y_coord][placed_shingle.x_coord] == 0:
                                self.install_shingle_target = placed_shingle
                                self.robot_state = RobotState.INSTALL_SHINGLE
                                # signel intention to move
                                inchworm_occ[placed_shingle.y_coord][placed_shingle.x_coord] = 1
                                rospy.loginfo(
                                    f"inchworm {self.id} installing shingle")
                                installing = True
                                self.installing_status = 1
                            else:
                                self.make_state_move_to_depot()

                        
                        elif Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):

                            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.ee_to_move_to_shingle)

                        else:
                            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.ee_to_move_to_shingle)


                        # check to see if we know the best path exists & if that shingle is free, if so, initaite move there
                        if not installing:
                            self.ee_shingle_neighbor_index = 0


                            rospy.loginfo(self.ee_shingle_neighbors)
                            if (len(self.ee_shingle_neighbors) > 0 and 
                                self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.INSTALLED and 
                                inchworm_occ[self.ee_shingle_neighbors[0]["pos"][1]][self.ee_shingle_neighbors[0]["pos"][0]] == 0
                                ):

                                inchworm_occ[self.ee_shingle_neighbors[0]["pos"]
                                             [1]][self.ee_shingle_neighbors[0]["pos"][0]] = 1
                                rospy.logwarn("moving to target")
                                self.robot_state = RobotState.MOVE_TO_TARGET

                            # otherwise start probing the points
                            self.ee_shingle_neighbor_index = 0
                            # self.robot_state = RobotState.PROBE_SHINGLE

                    else:  # The inchworm needs to move a shingle
                        # move a shingle closer to the target
                        self.shingle_to_move = placed_shingle
                        placed_shingle_location = [
                            placed_shingle.x_coord, placed_shingle.y_coord]

                        top_foot_shingle_neighbors = self.ee_to_move_to_free_space(
                            self.get_ee_neighbors(self.top_foot_position), EE.BOTTOM_FOOT)
                        bottom_foot_shingle_neighbors = self.ee_to_move_to_free_space(
                            self.get_ee_neighbors(self.bottom_foot_position), EE.TOP_FOOT)
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

                        top_foot_neighbors = [
                            value for elem in top_foot_shingle_neighbors for value in elem.values()]
                        bottom_foot_neighbors = [
                            value for elem in bottom_foot_shingle_neighbors for value in elem.values()]

                        # if the placed shingle is in one neighbor list and not the other, move the other
                        if placed_shingle_location in top_foot_neighbors and placed_shingle_location not in bottom_foot_neighbors:
                            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.ee_to_move_to_free_space)
                            if len(top_foot_shingle_neighbors) > 0:
                                inchworm_occ[top_foot_shingle_neighbors[0]["pos"]
                                                [1]][top_foot_shingle_neighbors[0]["pos"][0]] = 1

                        elif placed_shingle_location in bottom_foot_neighbors and placed_shingle_location not in top_foot_neighbors:
                            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.ee_to_move_to_free_space)
                            if len(bottom_foot_shingle_neighbors) > 0:
                                inchworm_occ[bottom_foot_shingle_neighbors[0]["pos"]
                                                [1]][bottom_foot_shingle_neighbors[0]["pos"][0]] = 1

                        # for additional checks, make sure that both shingle lists have at least one entry
                        elif len(bottom_foot_shingle_neighbors) > 0 and len(top_foot_shingle_neighbors) > 0:
                            # if bottom_foot is farther away, move bottom_foot
                            if Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) > Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target):
                                self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.ee_to_move_to_free_space)
                                inchworm_occ[top_foot_shingle_neighbors[0]["pos"]
                                                [1]][top_foot_shingle_neighbors[0]["pos"][0]] = 1
                            # if top_foot is farther away, move top_foot
                            elif Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) < Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target):
                                self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.ee_to_move_to_free_space)
                                inchworm_occ[bottom_foot_shingle_neighbors[0]["pos"]
                                                [1]][bottom_foot_shingle_neighbors[0]["pos"][0]] = 1

                        else:
                            rospy.logwarn(
                                f"inchworm {self.id} could not figure out which end effector to use to move the shingle at {placed_shingle_location}")

                        inchworm_occ[placed_shingle_location[1]
                                        ][placed_shingle_location[0]] = 1
                        self.move_shingle_step = 1
                        self.ee_shingle_neighbor_index = 0
                        self.robot_state = RobotState.MOVE_SHINGLE
                        rospy.loginfo(f"inchworm {self.id} moving shingle")

                else:
                    # if the inchworm is not next to the shingle depot, move toward it, otherwise pick up a new shingle
                    if not self.next_to_shingle_depot(shingle_depots[0]):
                        self.make_state_move_to_depot()
                    else:
                        rospy.loginfo(
                            f"inchworm {self.id} picking up from depot")
                        self.robot_state = RobotState.PICKUP_SHINGLE_FROM_DEPOT

        return shingles, inchworm_occ, shingle_depots

    def run_action(self, shingles, inchworm_occ, shingle_depots, shingle_count):
        # this is where all movement will happen

        if self.robot_state == RobotState.PICKUP_SHINGLE_FROM_DEPOT:
            # place holder for now, shingle depot just spawns a new shingle in the persumed location

            rospy.sleep(Inchworm.DELAY)
            rospy.loginfo(f"shingle depot at {shingle_depots[0].get_location()}")
            # check if the shingle depot has placed a shingle in the new spot

            if shingles[shingle_depots[0].get_location() + 1][0] is not None:
                if shingle_depots[0].get_location() != len(self.roof)/self.roof_width: #  prevents the shingle depot from going off the top of the roof
                    shingle_depots[0].move_shingle_depot_up()
                if not self.next_to_shingle_depot(shingle_depots[0]):
                    self.make_state_move_to_depot()
            else:
                # otherwise create a new shingle at the location and mark it as placed
                shingles[shingle_depots[0].get_location() + 1][0], shingle_count = shingle_depots[0].create_shingle(False, shingle_count)
                shingles[shingle_depots[0].get_location() + 1][0].place_shingle(0, shingle_depots[0].get_location() + 1)

                self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_TO_TARGET:
            # look the ee shingle neighbors to determain which end effector is being moved to target
            if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == EE.BOTTOM_FOOT:
                # check if ee is at the target
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.bottom_foot_position:
                    self.move_bottom_foot(
                        self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                else:
                    inchworm_occ[self.old_bottom_foot[1]][self.old_bottom_foot[0]] = 0
                    self.bottom_foot_status = EEStatus.PLANTED
                    self.robot_state = RobotState.MAKE_DECISION
            else:
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.top_foot_position:
                    self.move_top_foot(
                        self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                else:
                    inchworm_occ[self.old_top_foot[1]][self.old_top_foot[0]] = 0
                    self.top_foot_status = EEStatus.PLANTED
                    self.robot_state = RobotState.MAKE_DECISION

        elif self.robot_state == RobotState.INSTALL_SHINGLE:
            if self.placed_shingle_is_valid(self.install_shingle_target):
                # double check that the placed shingle is a valid install 
                rospy.loginfo(
                    f"inchworm {self.id} is installing a shingle at {self.target}")
                # small state machine to allow for multi step install
                if self.installing_status == 1:
                    # move ee to position in air

                    # figure out which ee needs to move to get closer to target
                    if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.ee_to_move_to)
                        self.installing_status = 2

                    else:

                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.ee_to_move_to)


                        self.installing_status = 2
                # install the shingle and place the ee on the new shingle
                elif self.installing_status == 2: 
                    if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == EE.BOTTOM_FOOT:
                        if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.bottom_foot_position:
                            self.move_bottom_foot(
                                self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                        else:
                            shingles[self.install_shingle_target.y_coord][self.install_shingle_target.x_coord].shingle_status = ShingleStatus.INSTALLED

                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.INSTALLED)


                            rospy.sleep(Inchworm.DELAY)
                            rospy.loginfo(
                                f"inchworm {self.id} has installed a shingle at {[[self.install_shingle_target.y_coord],[self.install_shingle_target.x_coord]]}")
                            self.bottom_foot_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                            self.installing_status = 0

                    else:
                        if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.top_foot_position:
                            self.move_top_foot(
                                self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                        else:
                            shingles[self.install_shingle_target.y_coord][self.install_shingle_target.x_coord].shingle_status = ShingleStatus.INSTALLED

                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.INSTALLED)

                            rospy.sleep(Inchworm.DELAY)
                            rospy.loginfo(
                                f"inchworm {self.id} has installed a shingle at {[[self.install_shingle_target.y_coord],[self.install_shingle_target.x_coord]]}")
                            self.top_foot_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                            self.installing_status = 0

        elif self.robot_state == RobotState.PATROL_FRONTIER:
            # TODO: This is where we could implement moving along the frontier, just moving about
            self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_SHINGLE:
            rospy.sleep(Inchworm.DELAY)

            rospy.loginfo(
                f"inchworm {self.id} move shingle step of {self.move_shingle_step}")
            if self.move_shingle_step == 1:
                self.old_shingle_pos = [
                    self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == EE.BOTTOM_FOOT:
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.bottom_foot_position:
                        rospy.loginfo(
                            f"inchworm {self.id} moving bottom_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_bottom_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])

                        shingles[self.shingle_to_move.y_coord][self.shingle_to_move.x_coord] = None
                        self.shingle_to_move = self.shingle_to_move.pickup_shingle()
                    else:
                        self.move_shingle_step = 2

                else:
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.top_foot_position:
                        rospy.loginfo(
                            f"inchworm {self.id} moving top_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_top_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        shingles[self.shingle_to_move.y_coord][self.shingle_to_move.x_coord] = None
                        self.shingle_to_move = self.shingle_to_move.pickup_shingle()

                    else:
                        self.move_shingle_step = 2

            elif self.move_shingle_step == 2:
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == EE.BOTTOM_FOOT:
                    if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.bottom_foot_position:
                        self.move_bottom_foot(
                            self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                    else:
                        self.shingle_to_move = self.shingle_to_move.place_shingle(
                            self.bottom_foot_position[0], self.bottom_foot_position[1])

                        shingles[self.bottom_foot_position[1]
                                 ][self.bottom_foot_position[0]] = self.shingle_to_move

                        self.move_shingle_step = 3

                else:
                    if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.top_foot_position:
                        self.move_top_foot(
                            self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                    else:
                        self.shingle_to_move = self.shingle_to_move.place_shingle(
                            self.top_foot_position[0], self.top_foot_position[1])
                        shingles[self.top_foot_position[1]
                                 ][self.top_foot_position[0]] = self.shingle_to_move
                        self.move_shingle_step = 3

            else:

                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == EE.BOTTOM_FOOT:
                    self.move_bottom_foot(self.old_bottom_foot)
                    inchworm_occ[self.old_bottom_foot[1]][self.old_bottom_foot[0]] = 0
                    self.bottom_foot_status = EEStatus.PLANTED

                else:
                    self.move_top_foot(self.old_top_foot)
                    inchworm_occ[self.old_top_foot[1]][self.old_top_foot[0]] = 0

                    self.top_foot_status = EEStatus.PLANTED
                inchworm_occ[self.shingle_to_move.y_coord][self.shingle_to_move.x_coord] = 0
                inchworm_occ[self.old_shingle_pos[1]
                             ][self.old_shingle_pos[0]] = 0
                self.robot_state = RobotState.MAKE_DECISION
                self.move_shingle_step = 0

        elif self.robot_state == RobotState.PROBE_SHINGLE:
            # # TODO: should move the inchworm to the suposed location and probe it
            
            rospy.sleep(Inchworm.DELAY)
        elif self.robot_state == RobotState.MOVE_TO_DEPOT:
            
            self.target = [0, shingle_depots[0].get_location()]
            # figures out the position to move toward the shingle depot 
            # TODO:this should be include some path planning that works the same as moving toward the target
            if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.ee_to_move_to)
            else:

                self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.ee_to_move_to)
            self.robot_state = RobotState.MOVE_TO_TARGET


            # check to see if we know the best path exists & if that shingle is free, if so, initaite move there

            self.ee_shingle_neighbor_index = 0
            if (self.roof[self.ee_shingle_neighbors[0]["pos"][1] * self.roof_width + self.ee_shingle_neighbors[0]["pos"][0]] == 2 and
                    inchworm_occ[self.ee_shingle_neighbors[0]["pos"][1]][self.ee_shingle_neighbors[0]["pos"][0]] == 0):
                inchworm_occ[self.ee_shingle_neighbors[0]["pos"]
                             [1]][self.ee_shingle_neighbors[0]["pos"][0]] = 1

                self.robot_state = RobotState.MOVE_TO_TARGET

            # otherwise start probing the points, this does not work correctly rn, probobly not needed if path planning is good
            self.ee_shingle_neighbor_index = 0
        return shingles, inchworm_occ, shingle_depots, shingle_count

    # returns true if there is a shingle at that location
    def probe(self, location, inchworm_occ, shingles):
        inchworm_occ[location["pos"][1]][location["pos"][0]] = 1
        if location["ee"] == EE.BOTTOM_FOOT:
            self.move_bottom_foot(location["pos"])
            rospy.loginfo(f"inchworm {self.id} probing {location}")
        else:
            self.move_top_foot(location["pos"])
            rospy.loginfo(f"inchworm {self.id} probing {location}")

        if shingles[location["pos"][1]][location["pos"][0]] is not None and shingles[location["pos"][1]][location["pos"][0]].shingle_status == ShingleStatus.INSTALLED:
            return True
        else:
            inchworm_occ[location["pos"][1]][location["pos"][0]] = 0
            return False

    def decide_on_movement_to_shingle(self, ee_to_move, neighbor_funtion):
        if ee_to_move == EE.BOTTOM_FOOT:
            self.ee_shingle_neighbors = neighbor_funtion(
                self.get_ee_neighbors(self.top_foot_position), EE.BOTTOM_FOOT)
            self.ee_shingle_neighbors.sort(
                key=lambda x: Inchworm.dist(x["pos"], self.target))
            self.old_bottom_foot = self.bottom_foot_position
            rospy.loginfo(f"inchworm {self.id} moving bottom_foot")
        else:
            self.ee_shingle_neighbors = neighbor_funtion(
                self.get_ee_neighbors(self.bottom_foot_position), EE.TOP_FOOT)
            self.ee_shingle_neighbors.sort(
                key=lambda x: Inchworm.dist(x["pos"], self.target))
            self.old_top_foot = self.top_foot_position
            rospy.loginfo(f"inchworm {self.id} moving top_foot")
        self.ee_shingle_neighbor_index = 0



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
                self.get_ee_neighbors(self.bottom_foot_position, False), [])
        else:
            msg.bottom_foot_valid_neighbors = []

        if self.top_foot_status == EEStatus.PLANTED:
            msg.top_foot_valid_neighbors = sum(
                self.get_ee_neighbors(self.top_foot_position, False), [])
        else:
            msg.top_foot_valid_neighbors = []

        msg.roof = self.roof
        msg.roof_width = self.roof_width
        msg.shingle_depot_pos = self.shingle_depot_pos
        msg.header.stamp = rospy.Time.now()
        return msg

    '''
    TODO:
        - move all ee movements into a seperate function
        - remove magic fuctions
            - will require a refactor of roof
        - implement the hex coords
        - create algo that gets the current frontier - involves getting all valid shingle locations based on current roof
        - read and write info to roof
        - make use of class EEShingleStatus(Enum):
        - create some viz for target
        - actualy use inchworm helpers

        - path planning so that the inchworm does not move in a gready fasion, this should treat all other inchworms as obsticals but will only need to run one tick at a time
        - make a new function for choosing the target tile based on the list of valid shingle targets - this is how we can create different patterns


        sudo apt-get install python-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev
        pip install pygame==1.9.6
    
    
    '''
