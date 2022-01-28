#!/usr/bin/env python3

import rospy, rospkg, math


if __name__ == "__main__":
    rospy.init_node("roof_manager")


    rospack = rospkg.RosPack()
    # publish roof state
    # sub to update shingle topic
    # service call to do varius things to shingles
