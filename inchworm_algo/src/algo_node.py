#!/usr/bin/env python3

import rospy, sys, random

from roof import Roof
from inchworm import Inchworm, EEStatus
from std_msgs.msg import Int32, Bool, Int32MultiArray
from shingle import ShingleStatus
from inchworm_algo.msg import RoofState, InchwormsMsg
from inchworm_algo.srv import GetInchwormState, GetInchwormStateResponse

# Globals that can be accessed by ROS callbacks
r = None
inchworms = []
paused = False
ticks = 0

def rateCB(msg):
    global r
    r = rospy.Rate(msg.data)

def pauseCB(msg):
    global paused
    paused = msg.data

def handle_get_inchworm_state(req):
    return GetInchwormStateResponse(state=inchworms[req.inchworm_idx].to_message())

def spawn_inchworms(roof, inchworm_count, pat):
        inchworm_count = min(int(roof.width/2), inchworm_count)
        inchworms = []
        for inchworm_id in range(inchworm_count):
            inchworms.append(Inchworm(id=inchworm_id, bottom_foot_pos=[inchworm_id * 2, 0], top_foot_pos=[(inchworm_id*2) + 1, 0], width=roof.width, height=roof.height, top_foot_stat=EEStatus.PLANTED, pattern=pat))
            roof.inchworm_occ[0][inchworm_id * 2] = 1
            roof.inchworm_occ[0][(inchworm_id*2) + 1] = 1
        return inchworms


def update_inchworms(roof, inchworms):
    global ticks
    ticks += 1
    is_done = False
    random.shuffle(inchworms)
    if ticks % 1000 == 0:
        rospy.loginfo(ticks)
    for worm in inchworms:
        if worm is not None:
            worm.make_decision(roof)
    for worm in inchworms:
        end_shingle = roof.get_shingle(0, roof.height - 1)
        if end_shingle is not None and end_shingle.shingle_status == ShingleStatus.INSTALLED:
            is_done = True
        if worm is not None and not is_done:
            worm.run_action(roof)
    return is_done, roof, inchworms

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

    pattern = int(sys.argv[5])
    inchworms = spawn_inchworms(roof, inchworm_count, pattern)

    roof_pub = rospy.Publisher("algo/roof_state", RoofState, queue_size=1)
    algo_finished_pub = rospy.Publisher("/algo/ticks_elapsed", Int32MultiArray, queue_size=1)
    inchworm_pub = rospy.Publisher("algo/inchworms", InchwormsMsg, queue_size=1)

    rate_sub = rospy.Subscriber("algo/rate", Int32, rateCB)
    pause_sub = rospy.Subscriber("algo/pause", Bool, pauseCB)

    rospy.Service("algo/get_inchworm_state", GetInchwormState, handle_get_inchworm_state)

    r = rospy.Rate(hz)
    status = False
    rospy.sleep(2) # time it takes to startup the algo viz
    while not rospy.is_shutdown() and not status:
        if not paused:

            roof_msg = roof.to_message()
            for worm in inchworms:
                roof_msg.inchworms.append(worm.to_message())
            roof_pub.publish(roof_msg)
            status, roof, inchworms = update_inchworms(roof, inchworms)
            # inchworm_pub.publish(create_inchworms_msg(inchworms))
            if status:
                rospy.loginfo("roof has been shingled")
                finished_msg = Int32MultiArray()
                finished_msg.data = [len(inchworms), ticks]
                algo_finished_pub.publish(finished_msg)
        r.sleep()
