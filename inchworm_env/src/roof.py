from inchworm_env.src.shingle import Shingle
from inchworm_env.src.shingle import ShingleStatus
from inchworm_env.src.shingle import NeighborIndex

# all x and y are in array coords currently


class Roof():
    # these arrays are used to lookup the correct NeighborIndex based on the difference in array coords
    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    width = 0
    height = 0
    shingle_array = []

    def __init__(self, width, height):
        self.shingle_array = [[None] * width] * height
        pass

    def place_shingle(self, shingle, x, y):
        self.shingle_array[x][y] = shingle
        shingle.place_shingle(x, y, self)


    def pickup_shingle(self, x, y):
        shingle = self.shingle_array[x][y].pickup_shingle()
        self.shingle_array[x][y] = None
        return shingle

    def install_shingle(self, shingle, x, y):
        self.shingle_array[x][y] = shingle
        shingle.install_shingle(x, y, self)

    def get_shingle(self, x, y):
        return self.shingle_array[x][y]
    
    def set_shingle(self, x, y, shingle):
        self.shingle_array[x][y] = shingle
    
    def get_shingle_n_index(self, t_x, t_y, n_x, n_y):
        delta_x = t_x - n_x
        delta_y = t_y - n_y
        if t_x % 2 == 0:
            return self.EVEN_ROW_N_LOOKUP.index((delta_x, delta_y))
        else:
            return self.ODD_ROW_N_LOOKUP.index((delta_x, delta_y))




    def update_shingle_neighbor(self, target_shingle_x, target_shingle_y, shingle_n_x, shingle_n_y, n_id, n_status):
        # figure out what the neighbor is based on x, y
        target_shingle = self.shingle_array[target_shingle_x][target_shingle_y]
        n_index = self.get_shingle_n_index(target_shingle_x, target_shingle_y, shingle_n_x, shingle_n_y)
        target_shingle.update_neighbor(n_id, n_index, n_status)
        
        


                
