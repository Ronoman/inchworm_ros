from inchworm_env.src.shingle_depot import ShingleDepot
from shingle import Shingle, ShingleStatus, NeighborIndex


# all x and y are in array coords currently

from inchworm_algo.msg import ShingleMsg, RoofState
import rospy

class Roof():
    # these arrays are used to lookup the correct NeighborIndex based on the difference in array coords
    # rows are 0 indexed
    # 0 starts on the left hand side of the roof
    # half shingles are on the right side of even rows and on the left side of odd rows
    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    width = 0
    height = 0
    shingle_array = [[]]
    shingle_depots = []
    inchworms = []
    shingle_count = -1

    def __init__(self, width, height, dual_side_depots):
        self.shingle_array = [ [None]*width for i in range(height)]
        self.width = width
        self.height = height
        self.spawn_first_row()
        self.spawn_depots(dual_side_depots)
        pass

    def place_shingle(self, shingle, x, y):
        self.shingle_array[y][x] = shingle
        shingle.place_shingle(x, y, self)


    def pickup_shingle(self, x, y):
        shingle = self.shingle_array[y][x].pickup_shingle()
        self.shingle_array[y][x] = None
        return shingle

    def install_shingle(self, shingle, x, y):
        self.shingle_array[y][x] = shingle
        shingle.install_shingle(x, y, self)

    def get_shingle(self, x, y):
        return self.shingle_array[y][x]
    
    def set_shingle(self, x, y, shingle):
        self.shingle_array[y][x] = shingle
    
    def get_shingle_n_index(self, t_x, t_y, n_x, n_y):
        delta_x = t_x - n_x
        delta_y = t_y - n_y
        if t_x % 2 == 0:
            return self.EVEN_ROW_N_LOOKUP.index((delta_x, delta_y))
        else:
            return self.ODD_ROW_N_LOOKUP.index((delta_x, delta_y))


    def spawn_first_row(self):
        rospy.loginfo("spawining first row of shingles")
        for i in range(self.width):
            is_half_shingle = False
            if i + 1 == self.width:
                is_half_shingle == True
            new_shingle = Shingle(i, is_half_shingle)
            new_shingle.install_shingle(i, 0)
            self.shingle_array[0][i] = new_shingle
        self.shingle_count = self.width
        
    def increment_shingle_count(self):
        self.shingle_count += 1
        return self.shingle_count


    def update_shingle_neighbor(self, target_shingle_x, target_shingle_y, shingle_n_x, shingle_n_y, n_id, n_status):
        # figure out what the neighbor is based on x, y 
        target_shingle = self.shingle_array[target_shingle_y][target_shingle_x]
        n_index = self.get_shingle_n_index(target_shingle_x, target_shingle_y, shingle_n_x, shingle_n_y)
        target_shingle.update_neighbor(n_id, n_index, n_status)
        

    def spawn_depots(self, spawn_opposite_side):
        self.shingle_depots.append(ShingleDepot(self, False))
        if spawn_opposite_side:
            self.shingle_depots.append(ShingleDepot(self, True))
        pass

    def move_shingle_depot(self, opposite_side):
        if opposite_side:
            self.shingle_depot[1].move_shingle_depot_up()
        else:
            self.shingle_depots[0].move_shingle_depot_up()

    def spawn_inchworms(num):
        pass
        
    def update_inchworms():
        pass

    

    def to_message(self):
        # TODO: include inchworms & shingle depot in message
        roof_state = RoofState()
        roof_state.width = self.width
        roof_state.height = self.height
        shingle_array_temp = []
        # this converts the shingle array in the roof to an occupancy grid
        for i in range(self.width):
            shingle_array_temp.extend(self.shingle_array[i])
        shingle_array_msg = []
        # rospy.loginfo(self.shingle_array)

        for shingle in shingle_array_temp:
            if shingle is None:
                shingle_array_msg.append(0)
            elif shingle.shingle_status == ShingleStatus.PLACED:
                shingle_array_msg.append(1)
            elif shingle.shingle_status == ShingleStatus.INSTALLED:
                shingle_array_msg.append(2)
            else:
                shingle_array_msg.append(-1) # this should not happen, if it does there is an error some were
        roof_state.shingles = shingle_array_msg
        # second depot is the one on the right
        if len(self.shingle_depots) == 2:
            roof_state.depot_positions = [self.shingle_depots[0].get_location(), self.shingle_depots[1].get_location()]
        else:
            roof_state.depot_positions = [self.shingle_depots[0].get_location()]
        inchworm_states = []
        for inchworm in self.inchworms:
            inchworm_states.append(inchworm.to_message())
        roof_state.header.stamp = rospy.Time.now()
        return roof_state



                
