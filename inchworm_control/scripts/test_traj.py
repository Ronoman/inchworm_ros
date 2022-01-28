#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import JointState

if __name__ == "__main__":
    rospy.init_node("test_traj")

    state_pub = rospy.Publisher("joint_state_test", JointState, queue_size=1)

    zero_pose = JointState()
    zero_pose.name = ["link1_to_foot", "link2_to_link1", "link3_to_link2", "link3_to_link4", "link4_to_foot"]
    zero_pose.position = [0]*5

    for i in range(10):
        zero_pose.header.stamp = rospy.Time.now()
        state_pub.publish(zero_pose)
        rospy.sleep(1)

    joint = int(input("Motor (0-4): "))

    lower_bound = int(input("Start pose (deg): "))
    upper_bound = int(input("End pose (deg): "))

    total_time = 3
    timestep = 0.1

    print(f"Motion time: {total_time}s")
    print(f"Timestep: {timestep*1000}ms")

    pts = np.linspace(lower_bound, upper_bound, int(total_time/timestep))

    start = rospy.Time.now()

    input("Press enter to start trajectory")

    for i in range(len(pts)):
        state = JointState()

        state.name = ["link1_to_foot", "link2_to_link1", "link3_to_link2", "link3_to_link4", "link4_to_foot"]
        state.position = [0]*5

        state.position[joint] = pts[i]

        state_pub.publish(state)

        rospy.sleep(timestep)