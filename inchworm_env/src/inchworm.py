from enum import Enum

from inchworm_env.msg import InchwormMsg
# all x and y are in array coords currently


class EEStatus(Enum):
    PLANTED = 0
    IN_AIR_NO_SHINGLE = 1
    IN_AIR_WITH_SHINGLE = 2


class Behavior(Enum):
    SKELETON = 0

class Inchworm():
    id = -1
    ee1_position = [-1, -1]
    ee2_position = [-1, -1]
    ee1_status = EEStatus.PLANTED
    ee2_status = EEStatus.PLANTED
    ee1_shingle_pos = [-1, -1]
    ee2_shingle_pos = [-1, -1]
    roof = [[]]

    

    def __init__(self, id = -1, is_half_shingle = False):
        self.id = id # this value should not change once it is assigned
        self.on_frontier = False
        self.is_half_shingle = is_half_shingle


    def create_from_message(self, msg):
        self.id = msg.id
        self.is_placed = msg.is_placed
        self.x_coord = msg.x_coord
        self.y_coord = msg.y_coord
        self.neighbors_ids = msg.neighbors_ids
        self.neighbors_status = msg.neighbors_status
        self.on_frontier = msg.on_frontier
        self.is_half_shingle = msg.is_half_shingle
        self.shingle_status = msg.shingle_status
        return self



    def place_shingle(self):
        #do something
        return self

    def pickup_shingle(self):
        #do something
        return self
    
    def install_shingle(self):
        #do something
        return self

    # get ids of all the neighbors and have the roof update the status 
    # n_location is a NeighborIndex
    # honestly not sure if we want to use this but it should only be used by the robot
    def update_shingle(self):
        #do something
        return self

    def read_shingle(self):
        #do something
        return self

    def to_message(self):
        msg = InchwormMsg()
        msg.id = self.id
        msg.ee1_pos = self.ee1_position
        msg.ee2_pos = self.ee2_position
        msg.


        msg.x_coord = self.x_coord
        msg.y_coord = self.y_coord
        msg.neighbors_ids = self.neighbors_ids
        msg.neighbors_status = self.neighbors_status
        msg.on_frontier = self.on_frontier
        msg.is_half_shingle = self.is_half_shingle
        msg.shingle_status = self.shingle_status
        return msg
        