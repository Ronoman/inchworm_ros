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
            self.roof[x + y * self.roof_width] = 1
        #update end effector status?
        return self

    def move_e1():
        pass


    def move_e2():
        pass

    def run_one_tick():
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
        