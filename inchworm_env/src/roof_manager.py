#!/usr/bin/env python3

import rospy, rospkg, math, sys

from inchworm_env.msg import ShingleMsg, RoofState
from roof import Roof
from shingle import Shingle, ShingleStatus
from inchworm_env.srv import *


def placeShingle(req, roof):
    shingle = Shingle(req.shingle)
    # need to create responce message 
    responce = RequestPlaceShingleResponse()
    responce.returnStatus = False
    if (roof[req.x_coord][req.y_coord] == None):
        shingle.shingle_status = ShingleStatus.PLACED
        shingle.x_coord = req.x_coord
        shingle.y_coord = req.y_coord
        roof[req.x_coord][req.y_coord] = shingle
        responce.returnStatus = True
    return responce


def pickupShingle(req, roof):
    shingle = roof.get_shingle(req.x_coord, req.y_coord)
    pass

def installShingle(req, roof):
    pass

def update_shingle(req, roof):
    pass

def read_shingle(req, roof):
    pass

if __name__ == "__main__":
    rospy.init_node("roof_manager")

    roof_width = sys.argv[1]
    roof_height = sys.argv[2]

    roof = Roof(roof_width, roof_height)
    roof.spawn_first_row()

    rospack = rospkg.RosPack()
    # publish roof state

    roof_pub = rospy.Publisher("roof/roof_state", RoofState, queue_size=1)
    # sub to update shingle topic
    # service call to do varius things to shingles
    while not rospy.is_shutdown():
        roof_pub.publish(roof.to_message())
        pass


''' 
    services needed:
        place shingle service
        pick up placed shingle
        install shingle
        update shingle
        read shingle - not nessarly needed, could just make it parse the roof state, but this would make it closer to the actual robot functinality 
'''