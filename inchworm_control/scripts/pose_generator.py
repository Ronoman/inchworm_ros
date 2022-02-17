#!/usr/bin/env python3

import rospy, math

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

current_joint_states = None

ik_service_proxy = None

def jointStateCB(msg):
  global current_joint_states

  current_joint_states = msg

def computeIK(pose, group="ltr", guess=None, timeout=5.0):
  ik_req = GetPositionIKRequest()

  ik_req.ik_request.group_name = group

  guess_angles = guess
  if guess_angles is None:
    guess_angles = current_joint_states.position

  ik_req.ik_request.robot_state.joint_state.name = current_joint_states.name
  ik_req.ik_request.robot_state.joint_state.position = guess_angles

  ik_req.pose_stamped.header.frame_id = "world"
  ik_req.pose_stamped.header.stamp = rospy.Time.now()
  ik_req.pose_stamped.pose = pose

  ik_req.timeout = rospy.Duration(timeout)

  res = ik_service_proxy(ik_req)

  return (res.solution, res.error_code)

def idx_to_coord(index, width):
  return (index % width, math.floor(index / width))

def coord_to_idx(coord, width):
  return coord[0] + width*coord[1]

def main():
  rospy.init_node("pose_generator")

if __name__ == "__main__":
  for i in range(49):
    print(coord_to_idx(idx_to_coord(i, 7), 7))
    print(idx_to_coord(i, 7))

  # Define set of transforms we want relative to shingle magnet frames
    # coincident, aligned at an offset, ???

  # Determine the shingle coords we are currently on based on args or parameters

  # For each of the six neighbors
    # Get the pose of the shingle magnet link relative to world
    # For each in transforms
      # Run compute IK on shingle magnet pose + transform
        # (Repeat on failure for N times)
      # Save these joint angles and the mirror (for other EE)