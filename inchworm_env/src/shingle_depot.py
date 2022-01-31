#!/usr/bin/env python3

import rospy, rospkg, math, shingle, sys

from inchworm_env.msg import Shingle



class ShingleDepot():
    shingle_count = -1
    depot_position = 0

    def __init__(self, roof_width) -> None:
        self.shingle_count = roof_width
        self.depot_position = 0
    
    def increment_shingle_count(self):
        self.shingle_count += 1
        return self.shingle_count

    def get_shingle_count(self):
        return self.shingle_count

    def get_location(self):
        return self.depot_position
    
    def move_shingle_depot_up(self):
        self.depot_position += 1
        return self.depot_position

def createShingle(req, shingle_depot):
    shingle_count = shingle_depot.increment_shingle_count()
    new_shingle = Shingle(shingle_count, req.half_shingle)
    return inchworm_env.srv.RequestShingleResponse(new_shingle)
#   rospy_tutorials.srv.AddTwoIntsResponse(req.a + req.)

def moveShingleDepot(location, shingle_depot):
    if location > shingle_depot.depot_position:
        shingle_depot.move_shingle_depot_up()


    
if __name__ == "__main__":
    rospy.init_node("shingle_depot")
    rospack = rospkg.RosPack()

    roof_width = sys.argv[1]

    shingle_depot = ShingleDepot()

    s = rospy.Service('request_new_shingle', inchworm_env.srv.RequestShingle, lambda msg: createShingle(msg, shingle_depot))
    
    rospy.Subscriber("move_depot", int, lambda msg: moveShingleDepot(msg, shingle_depot))

    position_pub = rospy.Publisher("/shingle_depot/position", int, queue_size=1)
    last_depot_postion = shingle_depot.get_location()
    while not rospy.is_shutdown():
        if last_depot_postion != shingle_depot.get_location():
            position_pub.publish(shingle_depot.get_location())
            last_depot_postion = shingle_depot.get_location()
    # 
