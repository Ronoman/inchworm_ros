from enum import Enum
from pickletools import int4

from inchworm_algo.msg import InchwormMsg
import rospy, math
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
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
    MAKE_DECISION = 7


# TODO: WE ARE CURRENTLY IGNORRING HALF SHINGLES
class Inchworm():
    id = -1
    ee1_position = [-1, -1]
    ee2_position = [-1, -1]
    ee1_status = EEStatus.PLANTED
    ee2_status = EEStatus.PLANTED
    ee1_shingle_stat = EEShingleStatus.INSTALLED
    ee2_shingle_stat = EEShingleStatus.INSTALLED
    behavior = Behavior.SKELETON
    roof = [] #occupancy grid


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
        for i in range(width):
            self.roof[i] = ShingleStatus.INSTALLED.value
        self.shingle_depot_pos = shingle_depot_pos
        self.next_tick_time = rospy.Time.now()
        self.robot_state = RobotState.MAKE_DECISION
        self.target_ee1_pos = ee1_pos
        self.target_ee2_pos = ee2_pos
        self.ee_shingle_neighbor_index = 0
        self.ee_shingle_neighbors = []
        self.installing_status = 0



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
            shingle = shingle.place_shingle(self.ee1_position[0], self.ee1_position[1])
            roof.place_shingle(shingle, self.ee1_position[0], self.ee1_position[1])
            self.roof[self.ee1_position[0] + self.ee1_position[1] * self.roof_width] = 1
            self.ee1_shingle_stat = EEShingleStatus.PLACED
        else:
            shingle = shingle.place_shingle(self.ee2_position[0], self.ee2_position[1])
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
        shingle = shingle.pickup_shingle()
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
            shingle = shingle.install_shingle(self.ee1_position[0], self.ee1_position[1])
            roof.install_shingle(shingle, self.ee1_position[0], self.ee1_position[1])
            self.ee1_shingle_stat = EEShingleStatus.INSTALLED
        else:
            shingle = shingle.install_shingle(self.ee2_position[0], self.ee2_position[1])
            roof.install_shingle(shingle, self.ee2_position[0], self.ee2_position[1])
            self.ee2_shingle_stat = EEShingleStatus.INSTALLED
        
        return self

    
    def update_shingle(self, ee, shingle): # TODO: LOOK BACK AT THIS AND MAKE SURE IT MAKES SENSE
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
            self.roof[x + y * self.roof_width] = shingle.shingle_status.value
        #update end effector status?
        return self

    def move_ee1(self, new_pos):
        self.ee1_status = EEStatus.IN_AIR
        self.ee1_position = new_pos


    def move_ee2(self, new_pos):
        self.ee2_status = EEStatus.IN_AIR
        self.ee2_position = new_pos

    def rebuild_roof(self): # TODO: IF THIS STARTS TO MAKE THINGS SLOW, MAKE IT NOT RECUSIVE
        for i, occ in enumerate(self.roof):
            if occ == 1:
                x = i%self.roof_width
                y = int(i/self.roof_width)
                self.make_children_valid(x, y)
        pass

    
    def make_children_valid(self, x, y):
        self.roof[y * self.roof_width + x] = 1
        if y % 2 == 0: # handle an even row
            test_x = x + Inchworm.EVEN_ROW_N_LOOKUP[1][0]
            test_y = y + Inchworm.EVEN_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 0:
                self.make_children_valid(test_x, test_y)
            test_x = x + Inchworm.EVEN_ROW_N_LOOKUP[2][0]
            test_y = y + Inchworm.EVEN_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 0:
                self.make_children_valid(test_x, test_y)
        else:
            test_x = x + Inchworm.ODD_ROW_N_LOOKUP[1][0]
            test_y = y + Inchworm.ODD_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 0:
                self.make_children_valid(test_x, test_y)
            test_x = x + Inchworm.ODD_ROW_N_LOOKUP[2][0]
            test_y = y + Inchworm.ODD_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 0:
                self.make_children_valid(test_x, test_y)
        pass

    # TODO: MAKE THE FOLLOWING TWO FUNCTION NOT MAGIC
    def next_to_placed_shingle(self, pos, shingles):
        neighbors = self.get_ee_neighbors(pos)
        
        for n in neighbors:
            
            read_shingle = shingles[n[1]][n[0]]
            if read_shingle is not None:
                if read_shingle.shingle_status == ShingleStatus.PLACED:
                    return True
        return False


    def get_shingle_neighbors(self, pos, shingles):
        neighbors = self.get_ee_neighbors(pos)
        shingle_neighbors = []
        for n in neighbors:
            
            read_shingle = shingles[n[1]][n[0]]
            if read_shingle is not None:
                if read_shingle.shingle_status == ShingleStatus.PLACED:
                    shingle_neighbors.append(n)
        return shingle_neighbors

    def get_best_placed_shingle(self, shingles):
        neighbors = self.get_shingle_neighbors(self.ee1_position, shingles)
        neighbors.extend(self.get_shingle_neighbors(self.ee2_position, shingles))
        neighbors.sort(reverse=True, key = lambda x: Inchworm.dist(x, [0, 0]))
        shingle_to_move = neighbors[0]
        return shingles[shingle_to_move[1]][shingle_to_move[0]]

    def placed_shingle_is_valid(self, shingle):
        validity_count = 0
        if shingle.y_coord % 2 == 0:
            test_x = shingle.x_coord + Inchworm.EVEN_ROW_N_LOOKUP[1][0]
            test_y = shingle.y_coord + Inchworm.EVEN_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 2:
                validity_count += 1
            test_x = shingle.x_coord + Inchworm.EVEN_ROW_N_LOOKUP[2][0]
            test_y = shingle.y_coord + Inchworm.EVEN_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 2:
                validity_count += 1
            if shingle.x_coord == self.roof_width:
                validity_count += 1
        else :
            test_x = shingle.x_coord + Inchworm.ODD_ROW_N_LOOKUP[1][0]
            test_y = shingle.y_coord + Inchworm.ODD_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 2:
                validity_count += 1
            test_x = shingle.x_coord + Inchworm.ODD_ROW_N_LOOKUP[2][0]
            test_y = shingle.y_coord + Inchworm.ODD_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and self.roof[test_y * self.roof_width + test_x] == 2:
                validity_count += 1
            if shingle.x_coord == 0:
                validity_count += 1
        
        return validity_count >=2

    def choose_shingle_target(self, placed_shinge_loc): # TODO: this will be more complicated in the future - could encode behaviors in here 
        x_coord = -1
        y_coord = -1
        for i, val in enumerate(self.roof):
            if val != 2:
                x_coord = i%self.roof_width
                y_coord = int(i/self.roof_width)
                break
        return [x_coord, y_coord]

    def dist(ee_pos, target_pos):
        return math.sqrt((ee_pos[0] - target_pos[0])**2 +  (ee_pos[1] - target_pos[1])**2 )


    def next_to_shingle_depot(self, shingle_depot):
        rospy.loginfo(f"ee1 at {self.ee1_position}")
        rospy.loginfo(f"ee2 at {self.ee2_position}")
        if self.ee1_position[0] == 0:
            if self.ee1_position[1] == shingle_depot.get_location():
                return True
            elif self.ee1_position[1] > shingle_depot.get_location():
                self.shingle_depot_pos[0] = shingle_depot.get_location()
        if self.ee2_position[0] == 0:
            
            if self.ee2_position[1] == shingle_depot.get_location():
                return True
            elif self.ee2_position[1] > shingle_depot.get_location():
                self.shingle_depot_pos[0] = shingle_depot.get_location()

        return False

    def get_ee_neighbors(self, ee_pos):
        neighbor_pos = []
        if ee_pos[1] % 2 == 0: # even row lookup
            for n in Inchworm.EVEN_ROW_N_LOOKUP:
                new_neighbor_pos = [ee_pos[0] + n[0], ee_pos[1] + n[1]]
                if new_neighbor_pos != self.ee1_position and new_neighbor_pos != self.ee2_position:
                    neighbor_pos.append(new_neighbor_pos)
        else:
            for n in Inchworm.ODD_ROW_N_LOOKUP:
                new_neighbor_pos = [ee_pos[0] + n[0], ee_pos[1] + n[1]]
                if new_neighbor_pos != self.ee1_position and new_neighbor_pos != self.ee2_position:
                    neighbor_pos.append(new_neighbor_pos)
        return neighbor_pos    


    

    def run_one_tick(self, shingles, inchworm_occ, shingle_depots): 

        # general idea is that this contitional is run everytime the robot has to make a desision,
        if self.behavior == Behavior.SKELETON:
            rospy.loginfo(f"running inchworm {self.id} for one tick {self.robot_state}")
            if self.robot_state == RobotState.MAKE_DECISION: # currently this is the only action that can take multiple itterations
                # read the shingles at the current ee positions
                # I am assuming that everytime this loop is run, both feet will be on the ground and we will want to read both shingles
                # TODO: allow inchworms to read the full data from a shingle and rebuild based off of that
                # TODO: if the data read does not match the inchworms such that the shingles are out of date, update the shingle
                # TODO: add this to the state machine down below
                if shingles[self.ee1_position[0]][self.ee1_position[1]] is not None:
                    self.read_shingle(shingles[self.ee1_position[0]][self.ee1_position[1]])
                if shingles[self.ee2_position[0]][self.ee2_position[1]] is not None:
                    self.read_shingle(shingles[self.ee2_position[0]][self.ee2_position[1]])


                # rebuild the roof based on constraints
                self.rebuild_roof()
                # get the coords of the current frontier shingles
                # frontier_coords = self.find_frontier()

                self.avg_pos = [(self.ee1_position[0] + self.ee2_position[0])/2, (self.ee1_position[1] + self.ee2_position[1])/2]
                rospy.loginfo(f"inchworm {self.id} ee1 is next to placed shingles : {self.next_to_placed_shingle(self.ee1_position, shingles)}")
                rospy.loginfo(f"inchworm {self.id} ee2 is next to placed shingles : {self.next_to_placed_shingle(self.ee2_position, shingles)}")

                if self.next_to_placed_shingle(self.ee1_position, shingles) or self.next_to_placed_shingle(self.ee2_position, shingles):
                    
                    # get the placed shingles  TODO: CHANGE THIS NEED TO INCLUDE MULTIPLE PLACED SHINGLES
                    

                    placed_shingle = self.get_best_placed_shingle(shingles)

                    
                    self.target = self.choose_shingle_target(placed_shingle) # TODO: change this definition - this is where real behaviors will happen
                    rospy.loginfo(f"inchworm {self.id} set target at {self.target}")
                    rospy.loginfo(f"moving towards {self.target}")
                    installing = False
                    if Inchworm.dist(self.avg_pos, self.target) > Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target):
                        if Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target) == 0.0: # check if placed shingle is in the target position
                            if inchworm_occ[placed_shingle.y_coord][placed_shingle.x_coord] == 0:
                                self.install_shingle_target = placed_shingle
                                self.robot_state = RobotState.INSTALL_SHINGLE
                                inchworm_occ[placed_shingle.y_coord][placed_shingle.x_coord] = 1 # signel intention to move
                                rospy.loginfo(f"inchworm {self.id} installing shingle")
                                installing = True
                                self.installing_status = 1
                            else:
                                self.robot_state = RobotState.MOVE_TO_DEPOT # TODO: figure out something better to do here
                                rospy.loginfo(f"inchworm {self.id} moving to depot")

                        elif Inchworm.dist(self.ee1_position, self.target) > Inchworm.dist(self.ee2_position, self.target): # figure out which ee needs to move to get closer to target
                            self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee2_position), 'ee1')
                            self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                            self.old_ee1 = self.ee1_position
                            rospy.loginfo(f"inchworm {self.id} moving ee1")

                        else: 
                            self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee1_position), 'ee2')
                            self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                            self.old_ee2 = self.ee2_position
                            rospy.loginfo(f"inchworm {self.id} moving ee2")
                        # check to see if we know the best path exists & if that shingle is free, if so, initaite move there

                        if not installing:
                            self.ee_shingle_neighbor_index = 0
                            if (self.roof[self.ee_shingle_neighbors[0]["pos"][1] * self.roof_width  + self.ee_shingle_neighbors[0]["pos"][0]] == 2 and
                                    inchworm_occ[self.ee_shingle_neighbors[0]["pos"][1]][self.ee_shingle_neighbors[0]["pos"][0]] == 0):
                                inchworm_occ[self.ee_shingle_neighbors[0]["pos"][1]][self.ee_shingle_neighbors[0]["pos"][0]] = 1
                
                                
                                self.robot_state = RobotState.MOVE_TO_TARGET

                            # otherwise start probing the points
                            self.ee_shingle_neighbor_index = 0
                            self.robot_state = RobotState.PROBE_SHINGLE
                    else:
                        if Inchworm.dist([placed_shingle.x_coord, placed_shingle.y_coord], self.target) == 0:
                            self.robot_state = RobotState.INSTALL_SHINGLE
                            rospy.loginfo(f"inchworm {self.id} installing shingle")
                        else:
                            self.shingle_to_move = placed_shingle
                            placed_shingle_location = [placed_shingle.x_coord, placed_shingle.y_coord]
                            # TODO: claim both positions

                            if Inchworm.dist(self.ee1_position, placed_shingle_location) > Inchworm.dist(self.ee2_position, placed_shingle_location): # figure out which ee needs to move to get closer to target
                                self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee2_position), 'ee1')
                                self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                                self.old_ee1 = self.ee1_position
                                rospy.loginfo(f"inchworm {self.id} moving ee1")

                            else: 
                                self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee1_position), 'ee2')
                                self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                                self.old_ee2 = self.ee2_position
                                rospy.loginfo(f"inchworm {self.id} moving ee2")



                            self.move_shingle_step = 0
                            self.robot_state.MOVE_SHINGLE
                            rospy.loginfo(f"inchworm {self.id} moving shingle")

                    



                else :
                    if not self.next_to_shingle_depot(shingle_depots[0]):
                        self.target = [0, self.shingle_depot_pos[0]]
                        rospy.loginfo(f"inchworm {self.id} going to depot")
                    else:
                        rospy.loginfo(f"inchworm {self.id} picking up from depot")
                        self.robot_state = RobotState.PICKUP_SHINGLE_FROM_DEPOT
                    
        return shingles, inchworm_occ, shingle_depots
                

                    
    def ee_to_move_to(self, neighbors, ee_to_move):
        move_targets = []
        for n in neighbors:
            move_targets.append({"pos": n, "ee":ee_to_move})
        return move_targets


    def run_action(self, shingles, inchworm_occ, shingle_depots, shingle_count):
        # this is where all movement will happen
        action = {} 
        if self.robot_state == RobotState.PICKUP_SHINGLE_FROM_DEPOT:
            # place holder for now, shingle depot just spawns a new shingle in the persumed location
            rospy.loginfo(f"shingle depot at {shingle_depots[0].get_location()}")
            if shingles[shingle_depots[0].get_location() + 1][0] is not None:
                if shingle_depots[0].get_location() != len(self.roof)/self.roof_width:
                    shingle_depots[0].move_shingle_depot_up()
                if not self.next_to_shingle_depot(shingle_depots[0]):
                    self.robot_state = RobotState.MOVE_TO_DEPOT
            else:
                shingles[shingle_depots[0].get_location() + 1][0], shingle_count = shingle_depots[0].create_shingle(False, shingle_count)
                shingles[shingle_depots[0].get_location() + 1][0].place_shingle(0, shingle_depots[0].get_location() + 1)
                
                self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_TO_TARGET:
            if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == "ee1":
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.ee1_position:
                    self.move_ee1(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                else:
                    inchworm_occ[self.old_ee1[1]][self.old_ee1[0]] = 0
                    self.ee1_status = EEStatus.PLANTED
                    self.robot_state = RobotState.MAKE_DECISION
                    
            else:
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.ee2_position:
                    self.move_ee2(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                else:
                    inchworm_occ[self.old_ee2[1]][self.old_ee2[0]] = 0
                    self.ee2_status = EEStatus.PLANTED
                    self.robot_state = RobotState.MAKE_DECISION

             
        elif self.robot_state == RobotState.INSTALL_SHINGLE:
            if self.placed_shingle_is_valid(self.install_shingle_target):
                # move the ee to that location - for now self installing shingles is fine
                
                if self.installing_status == 1:
                    # move ee to position in air
                    if Inchworm.dist(self.ee1_position, self.target) > Inchworm.dist(self.ee2_position, self.target): # figure out which ee needs to move to get closer to target
                        self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee2_position), 'ee1')
                        self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                        self.old_ee1 = self.ee1_position
                        rospy.loginfo(f"inchworm {self.id} moving ee1")
                        self.installing_status = 2


                    else: 
                        self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee1_position), 'ee2')
                        self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                        self.old_ee2 = self.ee2_position
                        rospy.loginfo(f"inchworm {self.id} moving ee2")
                        self.installing_status = 2




                elif self.installing_status == 2:
                    if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == "ee1":
                        if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.ee1_position:
                            self.move_ee1(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                        else:
                            shingles[self.install_shingle_target.y_coord][self.install_shingle_target.x_coord].shingle_status = ShingleStatus.INSTALLED
                            self.roof[self.install_shingle_target.y_coord * self.roof_width + self.install_shingle_target.x_coord] = ShingleStatus.INSTALLED.value
                            inchworm_occ[self.old_ee1[1]][self.old_ee1[0]] = 0
                            self.ee1_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                            self.installing_status = 0
                            
                    else:
                        if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.ee2_position:
                            self.move_ee2(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                        else:
                            shingles[self.install_shingle_target.y_coord][self.install_shingle_target.x_coord].shingle_status = ShingleStatus.INSTALLED
                            self.roof[self.install_shingle_target.y_coord * self.roof_width + self.install_shingle_target.x_coord] = ShingleStatus.INSTALLED.value
                            inchworm_occ[self.old_ee2[1]][self.old_ee2[0]] = 0
                            self.ee2_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                            self.installing_status = 0


        elif self.robot_state == RobotState.PATROL_FRONTIER:
            # TODO: DO THE THING
            self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_SHINGLE:
            # TODO: THIS WILL NOT RENDER CURRENTLY, IT MOVES THE TILE INSTANTANEOUSLY
            # figure out location that we want to move it to
            # claim the location where the shingle is and the location where we are moving it to
            # decide which ee will pick it up - the one behind it
            # probe to make sure that the shingle is there
            # pick up shingle
            # move ee to the new location
            # place the shingle down - write new location to the shingle and the roof
            # move ee to the original location
            if self.move_shingle_step == 1:
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == "ee1":
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.ee1_position:
                        self.move_ee1([self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        shingles[self.shingle_to_move.y_coord][self.shingle_to_move.y_coord] = None
                        self.shingle_to_move.pickup_shingle()
                    else:
                        self.move_shingle_step = 2
                        
                else:
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.ee2_position:
                        self.move_ee2([self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        shingles[self.shingle_to_move.y_coord][self.shingle_to_move.y_coord] = None
                        self.shingle_to_move.pickup_shingle()


                    else:
                        self.move_shingle_step = 2

            elif self.move_shingle_step == 2:
                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == "ee1":
                    if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.ee1_position:
                        self.move_ee1(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                    else:
                        self.shingle_to_move.place_shingle(self.ee1_position[0], self.ee1_position[1])
                        shingles[self.ee1_position[1]][self.ee1_position[0]] = self.shingle_to_move

                        self.move_shingle_step = 3
                        
                else:
                    if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"] != self.ee2_position:
                        self.move_ee2(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"])
                    else:
                        self.shingle_to_move.place_shingle(self.ee1_position[0], self.ee1_position[1])
                        shingles[self.ee1_position[1]][self.ee1_position[0]] = self.shingle_to_move
                        self.move_shingle_step = 3

            else:

                if self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["ee"] == "ee1":
                    self.move_ee1(self.old_ee1)
                    inchworm_occ[self.old_ee1[1]][self.old_ee1[0]] = 1 # TODO: clear the inchworm ooc here
                    self.ee1_status = EEStatus.PLANTED
                        
                else:
                    self.move_ee2(self.old_ee2)
                    inchworm_occ[self.old_ee2[1]][self.old_ee2[0]] = 1 # TODO: clear the inchworm ooc here
                    self.ee2_status = EEStatus.PLANTED
                self.robot_state = RobotState.MAKE_DECISION
                self.move_shingle_step = 0
                






            self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.PROBE_SHINGLE:
            
            if inchworm_occ[self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"][1]][self.ee_shingle_neighbors[self.ee_shingle_neighbor_index]["pos"][0]] == 0:
                if self.probe(self.ee_shingle_neighbors[self.ee_shingle_neighbor_index], inchworm_occ, shingles):
                    self.robot_state = RobotState.MOVE_TO_TARGET
                  
                self.ee_shingle_neighbor_index += 1
                if self.ee_shingle_neighbor_index >= len(self.ee_shingle_neighbors):
                    rospy.logwarn(f"inchworm {self.id} is trying to move and found no shingles that it could move it towards it's goal")
                    if self.ee_shingle_neighbors[0]["ee"] == "ee1":
                        self.ee1_position = self.old_ee1
                        inchworm_occ[self.ee1_position[1]][self.ee1_position[0]] = 1
                    else:
                        self.ee2_position = self.old_ee2
                        inchworm_occ[self.ee2_position[1]][self.ee2_position[0]] = 1
                    self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_TO_DEPOT:
            self.target = [0, shingle_depots[0].get_location()]
            if Inchworm.dist(self.ee1_position, self.target) > Inchworm.dist(self.ee2_position, self.target): # figure out which ee needs to move to get closer to target
                self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee2_position), 'ee1')
                self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                self.old_ee1 = self.ee1_position
                rospy.loginfo(f"inchworm {self.id} moving ee1")

            else: 
                self.ee_shingle_neighbors = self.ee_to_move_to(self.get_ee_neighbors(self.ee1_position), 'ee2')
                self.ee_shingle_neighbors.sort(key = lambda x: Inchworm.dist(x["pos"], self.target))
                self.old_ee2 = self.ee2_position
                rospy.loginfo(f"inchworm {self.id} moving ee2")
            # check to see if we know the best path exists & if that shingle is free, if so, initaite move there

            
            self.ee_shingle_neighbor_index = 0
            if (self.roof[self.ee_shingle_neighbors[0]["pos"][1] * self.roof_width  + self.ee_shingle_neighbors[0]["pos"][0]] == 2 and
                    inchworm_occ[self.ee_shingle_neighbors[0]["pos"][1]][self.ee_shingle_neighbors[0]["pos"][0]] == 0):
                inchworm_occ[self.ee_shingle_neighbors[0]["pos"][1]][self.ee_shingle_neighbors[0]["pos"][0]] = 1
                
                self.robot_state = RobotState.MOVE_TO_TARGET

            # otherwise start probing the points
            self.ee_shingle_neighbor_index = 0
            self.robot_state = RobotState.PROBE_SHINGLE
        return shingles, inchworm_occ, shingle_depots, shingle_count

    # returns true if there is a shingle at that location
    def probe(self, location, inchworm_occ, shingles):
        inchworm_occ[location["pos"][1]][location["pos"][0]] = 1
        if location["ee"] == "ee1":
            self.move_ee1(location["pos"])
        else:
            self.move_ee2(location["pos"])
        if shingles[location["pos"][1]][location["pos"][0]] is not None and shingles[location["pos"][1]][location["pos"][0]].shingle_status == ShingleStatus.INSTALLED:
            return True
        else:
            inchworm_occ[location["pos"][1]][location["pos"][0]] = 0
            return False



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
        

        - move shingle logic -- all of it
        
        - return stuff - because we are dealing with lists, I think it is alright
        sudo apt-get install python-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev
        pip install pygame==1.9.6
    
    
    '''