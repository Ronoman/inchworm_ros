#!/usr/bin/env python3

import rospy, rospkg, math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node("basic_control")

    pub = rospy.Publisher("/inchworm/arm_controller/command", JointTrajectory, queue_size=1)

    rospack = rospkg.RosPack()

    traj_path = rospack.get_path("inchworm_control") + "/trajectories/step.csv"

    print(traj_path)

    msg = JointTrajectory()
    msg.header.frame_id = "world"

    msg.joint_names = ["link1_to_foot", "link2_to_link1", "link3_to_link2", "link3_to_link4", "link4_to_foot"]

    traj_pts = []

    with open(traj_path) as traj:
        traj_pts = [[math.radians(float(angle)) for angle in pt.split(",")] for pt in traj.readlines()]

    print(traj_pts)

    for i,pt in enumerate(traj_pts):

        msg.points.append(JointTrajectoryPoint())

        msg.points[-1].positions = [0, -pt[0], pt[1], pt[2], 0]
        msg.points[-1].velocities = [0] * 5
        msg.points[-1].accelerations = [0] * 5

        msg.points[-1].time_from_start = rospy.Duration((i + 1) * 5)

    msg.header.stamp = rospy.Time.now()

    print(msg)
    input()

    pub.publish(msg)