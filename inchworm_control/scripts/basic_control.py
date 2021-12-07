#!/usr/bin/env python3

import rospy, math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node("basic_control")

    pub = rospy.Publisher("/inchworm/arm_controller/command", JointTrajectory, queue_size=1)

    while True:
        # We need to create a JointTrajectory to pass to the arm_controller
        msg = JointTrajectory()
        msg.header.frame_id = "world"

        # These are the five joints on the robot.
        msg.joint_names = ["link1_to_foot", "link2_to_link1", "link3_to_link2", "link3_to_link4", "link4_to_foot"]

        # To 
        msg.points.append(JointTrajectoryPoint())

        msg.points[0].positions = [0] * 5
        msg.points[0].velocities = [0] * 5
        msg.points[0].accelerations = [0] * 5

        msg.points[0].time_from_start = rospy.Duration(1.0)

        for i in range(5):

            pos = float(input(f"Joint {i} position (deg): "))
            msg.points[0].positions[i] = math.radians(pos)

        msg.header.stamp = rospy.Time.now()

        pub.publish(msg)