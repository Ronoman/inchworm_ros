from enum import Enum

from inchworm_env.msg import ShingleMsg
# all x and y are in array coords currently


class EdgeStatus(Enum):
    NO_EDGE = 0
    RIGHT = 1
    LEFT = 2
    BOTTEM = 3
    TOP = 4
    BOTTEM_RIGHT = 5
    BOTTEM_LEFT = 6
    TOP_RIGHT = 7
    TOP_LEFT = 8

class NeighborIndex(Enum):
    RIGHT = 0
    BOTTEM_RIGHT = 1
    BOTTEM_LEFT = 2
    LEFT = 3
    TOP_LEFT = 4
    TOP_RIGHT = 5


class ShingleStatus(Enum):
    UNINSTALLED = 0
    PLACED = 1
    INSTALLED = 2

class ShingleType(Enum):
    NORMAL_SHINGLE = 0
    HALF_SHINGLE = 1

class Shingle():

    id = -1
    x_coord = -1
    y_coord = -1
    neighbors_ids = [-1 * 6]
    neighbors_status = [-1 * 6]
    on_frontier = False
    edge = EdgeStatus.NO_EDGE
    is_half_shingle = False

    shingle_status = ShingleStatus.UNINSTALLED
    

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



    def place_shingle(self, x, y,):
        self.x_coord = x
        self.y_coord = y
        self.shingle_status = ShingleStatus.PLACED

    def pickup_shingle(self):
        self.x = -1
        self.y = -1
        self.shingle_status = ShingleStatus.UNINSTALLED
        

    
    def install_shingle(self, x, y):
        self.x_coord = x
        self.y_coord = y
        self.on_frontier = True
        self.shingle_status = ShingleStatus.INSTALLED


    # get ids of all the neighbors and have the roof update the status 
    # n_location is a NeighborIndex
    # honestly not sure if we want to use this but it should only be used by the robot
    def update_neighbor(self, id, n_locatation, n_status):
        self.neighbors_ids[n_locatation] = id
        self.neighbors_status[n_locatation] = n_status


    def to_message(self):
        msg = ShingleMsg()
        msg.id = self.id
        msg.x_coord = self.x_coord
        msg.y_coord = self.y_coord
        msg.neighbors_ids = self.neighbors_ids
        msg.neighbors_status = self.neighbors_status
        msg.on_frontier = self.on_frontier
        msg.is_half_shingle = self.is_half_shingle
        msg.shingle_status = self.shingle_status
        return msg
        