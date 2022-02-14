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

    def choose_shingle_target(self, placed_shinge_1, placed_shingle_2) # TODO: change this definition
        return None

    def dist(self, ee_pos, target):
        return 0

    def run_one_tick(self, roof): # the roof is passed in because the inchworm has to interact with the real roof
        if self.behavior == Behavior.SKELETON:
            rospy.loginfo(f"running inchworm {self.id} for one tick")
            if rospy.Time.now() >= self.next_tick_time:
                # read the shingles at the current ee positions
                # TODO: allow inchworms to read the full data from a shingle and rebuild based off of that
                # TODO: if the data read does not match the inchworms such that the shingles are out of date, update the shingle
                # TODO: add this to the state machine down below
                self.read_shingle(roof.get_shingle(self.ee1_position[0], self.ee1_position[1]))
                self.read_shingle(roof.get_shingle(self.ee2_position[0], self.ee2_position[1]))


                # rebuild the roof based on constraints
                self.rebuild_roof()
                # get the coords of the current frontier shingles

                frontier_coords = self.find_frontier()

                # check if either ee is on the frontier, if it is not, move towards shingle depot
                if self.ee1_position in frontier_coords or self.ee2_position in frontier_coords:
                    # move towards shingle depot
                    pass

                elif self.next_to_placed_shingle(self.ee1_position) or self.next_to_placed_shingle(self.ee2_position):
                    
                    # get the placed shingles
                    ee1_placed_shingle = None
                    ee2_placed_shingle = None


                    if self.placed_shingle_is_valid(ee1_placed_shingle): # TODO: logic in here will define behaviors
                        # install ee1
                        pass
                    elif self.placed_shingle_is_valid(ee2_placed_shingle): # TODO: logic in here will define behaviors
                        # install ee2
                        pass
                    else: # TODO: logic in here will define behaviors
                        target = self.choose_shingle_target(ee1_placed_shingle, ee2_placed_shingle) # TODO: change this definition
                        
                        if self.dist(self.ee1_position, target) < self.dist(self.ee2_position, target): # TODO: logic needs to change here if we need to pick up a shingle
                            # move ee1
                            # check if we need to pickup a shingle
                            pass
                        else: 
                            # move ee2
                            # check if we need to pickup a shingle
                            pass
                        pass

                        
                    pass

                else:
                    # move toward nearest shingle depot
                    pass


                   
                




                pass
            '''
            TODO:
                tick if done with action
                read both of the shingles 
                    rebuild your roof based on reads 
                find all frontier shingles
                decide on place to place a shingle
                if there is a placed shingle next to one of the end effectors, pick it up
                otherwise, move towards the nearest shingle depot


                When moving to a new shingle, assume that all empty shingle locations might have something there,
                    test to see if there is something there
                        rebuild roof if there is something there
            '''

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
        