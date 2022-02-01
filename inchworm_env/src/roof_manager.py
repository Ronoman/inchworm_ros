#!/usr/bin/env python3

import rospy, rospkg, math, sys

from inchworm_env.msg import ShingleMsg, RoofState
from roof import Roof
from shingle import Shingle, ShingleStatus
from inchworm_env.srv import *


def place_shingle(req, roof):
    shingle = Shingle().create_from_message(req.shingle)
    responce = PlaceShingleResponse()
    responce.returnStatus = False
    if (roof[req.x_coord][req.y_coord] == None):
        shingle.place_shingle(req.x_coord, req.y_coord)
        roof[req.x_coord][req.y_coord] = shingle
        responce.returnStatus = True
    return responce


def pickup_shingle(req, roof):
    shingle = roof.get_shingle(req.x_coord, req.y_coord)
    response = PickupShingleResponse()
    response.returnStatus = False
    if req.shingle_id == shingle.id and shingle.shingle_status == ShingleStatus.PLACED:
        shingle.pickupShingle()
        response.shingle = shingle.to_message()
        response.returnStatus = True
    return response

def install_shingle(req, roof):
    shingle = Shingle().create_from_message(req.shingle)
    # need to create responce message 
    responce = InstallShingleResponse()
    responce.returnStatus = False
    if (roof[req.x_coord][req.y_coord] == None):
        shingle.install_shingle(req.x_coord, req.y_coord)
        roof[req.x_coord][req.y_coord] = shingle
        responce.returnStatus = True
    return responce

# this just checks to see if the updated shingle has the same id and 
# the same type and then overwrites the old shingle
# must update an installed shingle
def update_shingle(req, roof):
    updated_shingle = Shingle().create_from_message(req.shingle)
    current_shingle = roof[updated_shingle.x_coord][updated_shingle.y_coord]
    response = UpdateShingleResponse()
    response.returnStatus = False
    if (updated_shingle.id == current_shingle.id and 
        updated_shingle.is_half_shingle == current_shingle.is_half_shingle and 
        updated_shingle.shingle_status == current_shingle.shingle_status):

        roof[updated_shingle.x_coord][updated_shingle.y_coord] == updated_shingle
        response.returnStatus = True
    return response




def read_shingle(req, roof):
    shingle = roof[req.x_coord][req.y_coord]
    response = ReadShingleResponse()
    response.shingle = shingle.to_message()
    return response


if __name__ == "__main__":
    rospy.init_node("roof_manager")

    roof_width = int(sys.argv[1])
    roof_height = int(sys.argv[2])
    hz = 10
    if len(sys.argv) == 4:
        hz = int(sys.argv[3])

    roof = Roof(roof_width, roof_height)
    roof.spawn_first_row()

    # publish roof state

    roof_pub = rospy.Publisher("/roof/roof_state", RoofState, queue_size=1)

    # create services

    read_shingle_service = rospy.Service('/read_shingle', ReadShingle, lambda msg: read_shingle(msg, roof))
    update_shingle_service = rospy.Service('/update_shingle', UpdateShingle, lambda msg: update_shingle(msg, roof))
    install_shingle_service = rospy.Service('/install_shingle', InstallShingle, lambda msg: install_shingle(msg, roof))
    pickup_shingle_service = rospy.Service('/pickup_shingle', PickupShingle, lambda msg: pickup_shingle(msg, roof))
    place_shingle_service = rospy.Service('/place_shingle', PlaceShingle, lambda msg: place_shingle(msg, roof))

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        roof_pub.publish(roof.to_message())
        r.sleep()
        

