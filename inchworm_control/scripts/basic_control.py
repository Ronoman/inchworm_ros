#!/usr/bin/env python3

import rospy, math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node("basic_control")

    pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)

    while True:
        # We need to create a JointTrajectory to pass to the position_trajectory_controller
        msg = JointTrajectory()
        msg.header.frame_id = "world"

        # These are the five joints on the robot.
        msg.joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

        # To 
        msg.points.append(JointTrajectoryPoint())

        msg.points[0].positions = [0] * 5
        msg.points[0].velocities = [0] * 5
        msg.points[0].accelerations = [0] * 5

        msg.points[0].time_from_start = rospy.Duration(1.0)

        for i in range(5):

            pos = float(input(f"Joint {i} position (rad): "))
            msg.points[0].positions[i] = (pos)

        msg.header.stamp = rospy.Time.now()

        pub.publish(msg)