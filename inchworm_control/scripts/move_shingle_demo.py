#!/usr/bin/env python3

import sys
import math
import rospy
import tf2_ros
import moveit_commander

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import JointState

from inchworm_hw_interface.msg import MagnetState

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

def goToPose(pose, wait=True, tries=5):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "world"
    goal_pose.header.stamp = rospy.Time.now()

    goal_pose.pose = pose

    goal_pub.publish(goal_pose)

    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)
    group.set_pose_target(goal_pose)

    foundPlan = False
    attempts = 0
    while not foundPlan or attempts > tries:
        plan = group.plan()
        if plan[0]:
            foundPlan = True
        else:
            attempts += 1

    if attempts > tries:
        rospy.logerr("Failed to find plan, quitting.")
        rospy.logwarn(pose)
        sys.exit()
    
    group.execute(plan[1], wait=wait)

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_ee")

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    js_sub = rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB, queue_size=1)

    goal_pub = rospy.Publisher("/inchworm/next_goal", PoseStamped, queue_size=1)
    traj_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)

    mag_state_pub = rospy.Publisher("/inchworm/magnet_states", MagnetState, queue_size=1)

    robot = moveit_commander.RobotCommander()

    group_name = "ltr"
    group = moveit_commander.MoveGroupCommander(group_name)

    while current_joint_states is None:
        rospy.sleep(1)

    trans = getTransform("world", "iw_foot_top", buffer, listener).transform
    (r, p, y) = transToRPY(trans)

    print(f"Reference frame: {group.get_planning_frame()}")
    print(f"End effector: {group.get_end_effector_link()}")

    print(f"Current orientation (deg):\n\tRoll: {r:.2f}\n\tPitch: {p:.2f}\n\tYaw: {y:.2f}")

    trans = getTransform("world", "shingle/female_0", buffer, listener).transform
    (r, p, y) = transToRPY(trans)

    # Rotate 180 about Z, start above the shingle
    # y = (y + 180) % 360
    quat = rpyToQuat(r, p, y)
    trans.translation.z += 0.05

    rot = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    pose = Pose(position=trans.translation, orientation=rot)

    rospy.logwarn("Going to approach pose")
    goToPose(pose)

    rospy.logwarn("Disabling magnet")
    top_disabled = MagnetState(magnet1=True, magnet2=False)
    mag_state_pub.publish(top_disabled)

    # Drop the EE back down
    pose.position.z -= 0.02
    
    rospy.logwarn("Approaching shingle")
    goToPose(pose)

    # Enable magnets
    rospy.logwarn("Enabling magnets")
    both_enabled = MagnetState(magnet1=True, magnet2=True)
    mag_state_pub.publish(both_enabled)

    rospy.sleep(5)

    rospy.logwarn("Moving away")
    pose.position.z += 0.03
    goToPose(pose)