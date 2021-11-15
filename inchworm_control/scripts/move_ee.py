#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState

current_joint_states = None

def jointStateCB(msg):
    global current_joint_states

    current_joint_states = msg


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_ee")

    js_sub = rospy.Subscriber("inchworm/joint_states", JointState, queue_size=1)

    robot = moveit_commander.RobotCommander

    group_name = "ltr"
    group = moveit_commander.MoveGroupCommander(group_name)

    print(f"Reference frame: {group.get_planning_frame()}")
    print(f"End effector: {group.get_end_effector_link()}")

    while current_joint_states is None:
        rospy.sleep(1)

    ik_req = GetPositionIKRequest()

    ik_req.group_name = group_name
    ik_req.robot_state.joint_state = current_joint_states
    ik_req.avoid_collisions = True
    ik_req.ik_link_name = "foot"

    print("Following are relative to reference frame.")
    print("Assuming current EE orientation\n")

    x = float(input("X: "))
    y = float(input("X: "))
    z = float(input("X: "))