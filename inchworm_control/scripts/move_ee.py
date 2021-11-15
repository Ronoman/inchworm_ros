#!/usr/bin/env python3

import sys
import math
import rospy
import tf2_ros
import moveit_commander

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState

current_joint_states = None

def jointStateCB(msg):
    global current_joint_states

    current_joint_states = msg

def getTransform(frame_from, frame_to, buffer, listener):
    have_transform = False
    trans = None

    while not have_transform:
        try:
            trans = buffer.lookup_transform(frame_from, frame_to, rospy.Time(0))
            have_transform = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            print(f"Lookup from {frame_from} to {frame_to} failed with error:")
            print(err)

            rospy.sleep(1.0)
            continue

    return trans

def transToRPY(trans):
    quat = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]

    (r, p, y) = [math.degrees(n) for n in euler_from_quaternion(quat)]

    return r,p,y

def rpyToQuat(roll, pitch, yaw):
    # Assume incoming is in degrees
    roll, pitch, yaw = [math.radians(n) for n in (roll, pitch, yaw)]

    quat = quaternion_from_euler(roll, pitch, yaw)

    return quat

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_ee")

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    js_sub = rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB, queue_size=1)
    goal_pub = rospy.Publisher("/inchworm/next_goal", PoseStamped, queue_size=1)

    robot = moveit_commander.RobotCommander()

    group_name = "ltr"
    group = moveit_commander.MoveGroupCommander(group_name)

    while current_joint_states is None:
        rospy.sleep(1)

    trans = getTransform("foot", "other_foot", buffer, listener).transform
    (r, p, y) = transToRPY(trans)

    print(f"Reference frame: {group.get_planning_frame()}")
    print(f"End effector: {group.get_end_effector_link()}")

    print(f"Current orientation (deg):\n\tRoll: {r:.2f}\n\tPitch: {p:.2f}\n\tYaw: {y:.2f}")

    ik_req = GetPositionIKRequest()

    ik_req.ik_request.group_name = group_name
    ik_req.ik_request.robot_state.joint_state = current_joint_states
    ik_req.ik_request.avoid_collisions = True
    ik_req.ik_request.ik_link_name = "other_foot"

    print("Following are relative to reference frame.")

    print("Position in meters:")
    x = float(input("X: "))
    y = float(input("Y: "))
    z = float(input("Z: "))

    print("Rotation in degrees:")
    roll = float(input("Roll: "))
    pitch = float(input("Pitch: "))
    yaw = float(input("Yaw: "))

    quat = rpyToQuat(roll, pitch, yaw)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = group.get_planning_frame()
    goal_pose.header.stamp = rospy.Time.now()

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    goal_pose.pose.orientation.x = quat[0]
    goal_pose.pose.orientation.y = quat[1]
    goal_pose.pose.orientation.z = quat[2]
    goal_pose.pose.orientation.w = quat[3]

    ik_req.ik_request.pose_stamped = goal_pose
    ik_req.ik_request.timeout = rospy.Duration(5)

    goal_pub.publish(goal_pose)
    print(goal_pose)

    ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    res = ik_srv(ik_req)

    print(res)