#!/usr/bin/env python3

import rospy, math, shingle, sys, std_msgs

from inchworm_algo.msg import ShingleMsg, RoofState
from roof import Roof
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
from inchworm import Inchworm, EEStatus
from inchworm_algo.srv import *

def spawn_inchworms(roof, inchworm_count):
        inchworm_count = min(int(roof.width/2), inchworm_count)
        inchworms = []
        for inchworm_id in range(inchworm_count):
            inchworms.append(Inchworm(id=inchworm_id, bottom_foot_pos=[inchworm_id * 2, 0], top_foot_pos=[(inchworm_id*2) + 1, 0], width=roof.width, height=roof.height, top_foot_stat=EEStatus.PLANTED))
            roof.inchworm_occ[0][inchworm_id * 2] = 1
            roof.inchworm_occ[0][(inchworm_id*2) + 1] = 1
        return inchworms


def update_inchworms(roof, inchworms):
    for worm in inchworms:
        if worm is not None:
            worm.run_one_tick(roof)
    for worm in inchworms:
        if worm is not None:
            worm.run_action(roof)
    return roof, inchworms

if __name__ == "__main__":
    rospy.init_node("algo_node")

    roof_width = int(sys.argv[1])
    roof_height = int(sys.argv[2])

    hz = 5
    inchworm_count = int(sys.argv[3])
    if len(sys.argv) >= 4:
        hz = int(sys.argv[4])


    roof = Roof(roof_width, roof_height, False)
    
    inchworms = spawn_inchworms(roof, inchworm_count)

    # publish roof state

    roof_pub = rospy.Publisher("/algo/roof_state", RoofState, queue_size=1)


    r = rospy.Rate(hz)
    rospy.sleep(2) # time it takes to startup the algo viz
    while not rospy.is_shutdown():
        roof_pub.publish(roof.to_message())
        roof, inchworms = update_inchworms(roof, inchworms)
        r.sleep()



    '''
    TODO: 
        - refactor roof so that it does not include inchworms
        - get rid of all magic functions
            - this will require fixing reading shingles & storage of local roof copies
            - will also require a working probe roof subtask || making sure that when a inchworm installs a shingle it rewrites all neighbors
        - bundle subtasks that exist in the state machine into functions
        - refactor the run_one_tick to make more sense
        - change all distance & neighbor functions to use a hex grid lib
        - have rules for avoiding & escaping deadlock
        - better collision avoidence?
    
    '''
        