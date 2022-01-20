#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import JointState

if __name__ == "__main__":
    rospy.init_node("test_traj")

    state_pub = rospy.Publisher("joint_state_test", JointState, queue_size=1)

    zero_pose = JointState()
    zero_pose.name = ["j0", "j1", "j2", "j3", "j4"]
    zero_pose.position = [0]*5

    for i in range(10):
        zero_pose.header.stamp = rospy.Time.now()
        state_pub.publish(zero_pose)
        rospy.sleep(0.1)

    lower_bound = 30
    upper_bound = 0

    total_time = 3
    timestep = 0.1

    pts = np.linspace(lower_bound, upper_bound, int(total_time/timestep))

    start = rospy.Time.now()

    input("Press enter to start trajectory")

    for i in range(len(pts)):
        state = JointState()

        state.name = ["j0", "j1", "j2", "j3", "j4"]
        state.position = [0]*5

        state.position[1] = pts[i]
        state.position[2] = pts[i]
        state.position[3] = pts[i]

        state_pub.publish(state)

        rospy.sleep(timestep)