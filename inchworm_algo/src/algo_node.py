#!/usr/bin/env python3

import rospy, math, shingle, sys, std_msgs

from inchworm_algo.msg import ShingleMsg, RoofState
from roof import Roof
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
from inchworm_algo.srv import *




if __name__ == "__main__":
    rospy.init_node("algo_node")

    roof_width = int(sys.argv[1])
    roof_height = int(sys.argv[2])
    hz = 5
    inchworm_count = int(sys.argv[3])
    if len(sys.argv) == 4:
        hz = int(sys.argv[4])
    

    roof = Roof(roof_width, roof_height, False, inchworm_count)
    


    # publish roof state

    roof_pub = rospy.Publisher("/algo/roof_state", RoofState, queue_size=1)


    r = rospy.Rate(hz)
    rospy.sleep(2) # time it takes to startup the algo viz
    while not rospy.is_shutdown():
        roof_pub.publish(roof.to_message())
        roof.update_inchworms()
        r.sleep()
        