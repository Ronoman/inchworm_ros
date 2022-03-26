#!/usr/bin/env python3

import rospy, math, shingle, sys, std_msgs

from inchworm_algo.msg import ShingleMsg, RoofState, InchwormsMsg
from roof import Roof
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
from inchworm import Inchworm, EEStatus
from inchworm_algo.srv import *

from std_msgs.msg import Int32

r = None

def rateCB(msg):
    global r
    r = rospy.Rate(msg.data)

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
            worm.make_decision(roof)
    for worm in inchworms:
        if worm is not None:
            worm.run_action(roof)
    return roof, inchworms

def create_inchworms_msg(inchworms):
    inchworm_msg = InchwormsMsg()
    for inchworm in inchworms:
        inchworm_msg.inchworms.append(inchworm.to_message())
    inchworm_msg.header.stamp = rospy.Time.now()
    return inchworm_msg

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
    inchworm_pub = rospy.Publisher("/algo/inchworms", InchwormsMsg, queue_size=1)

    rate_sub = rospy.Subscriber("/algo/rate", Int32, rateCB)

    r = rospy.Rate(hz)
    rospy.sleep(2) # time it takes to startup the algo viz
    while not rospy.is_shutdown():
        roof_msg = roof.to_message()
        for worm in inchworms:
            roof_msg.inchworms.append(worm.to_message())
        roof_pub.publish(roof_msg)
        roof, inchworms = update_inchworms(roof, inchworms)
        # inchworm_pub.publish(create_inchworms_msg(inchworms))
        r.sleep()



        