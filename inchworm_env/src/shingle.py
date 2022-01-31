import enum
from enum import Enum
from tkinter import LEFT

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

class Shingle:

    id = -1
    x_coord = -1
    y_coord = -1
    neighbors_ids = [-1 * 6]
    neighbors_status = [-1 * 6]
    on_frontier = False
    edge = EdgeStatus.NO_EDGE
    shingle_type = ShingleType.NORMAL_SHINGLE
    shingle_status = ShingleStatus.UNINSTALLED
    

    def __init__(self, id, is_half_shingle):
        self.id = id
        self.on_frontier = False
        if is_half_shingle:
            self.shingle_type = ShingleType.HALF_SHINGLE
        pass

    def place_shingle(self, x, y,):
        self.x_corrd = x
        self.y_corrd = y
        self.shingle_status = ShingleStatus.PLACED

    def pickup_shingle(self):
        self.x = -1
        self.y = -1
        self.shingle_status = ShingleStatus.UNINSTALLED
        return self

    
    def install_shingle(self, x, y, roof):
        self.x_corrd = x
        self.y_corrd = y
        self.on_frontier = True
        self.shingle_status = ShingleStatus.INSTALLED


        # get ids of all the neighbors and have the roof update the status 
    # n_location is a NeighborIndex
    def update_neighbor(self, id, n_locatation, n_status):
        self.neighbors_ids[n_locatation] = id
        self.neighbors_status = n_status
        