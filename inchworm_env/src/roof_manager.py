#!/usr/bin/env python3

import rospy, rospkg, math, sys

from inchworm_env.msg import ShingleMsg, RoofState
from roof import Roof

if __name__ == "__main__":
    rospy.init_node("roof_manager")

    roof_width = sys.argv[1]
    roof_height = sys.argv[2]

    roof = Roof(roof_width, roof_height)
    roof.spawn_first_row()

    rospack = rospkg.RosPack()
    # publish roof state

    pub = rospy.Publisher("roof/roof_state", RoofState, queue_size=1)
    # sub to update shingle topic
    # service call to do varius things to shingles
    while not rospy.is_shutdown():
        pass
