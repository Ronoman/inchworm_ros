#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("basic_control")

    controller_pubs = []

    for i in range(5):
        controller_pubs.append(rospy.Publisher("/inchworm/joint_effort_controller_joint_{}/command".format(i+1), Float64, queue_size=5))

    while True:
        controller = int(input("Joint # (1-5): "))
        pos = float(input("New position: "))

        controller_pubs[controller-1].publish(pos)