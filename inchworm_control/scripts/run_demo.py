#!/usr/bin/env python3

import rospy, glob, rospkg, yaml, sys

from inchworm_hw_interface.msg import MagnetState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from traj_planner import TrajectoryPlanner

last_joint_state = None
trajectory_pub   = None
mag_state_pub    = None

def jointStateCB(msg):
    global last_joint_state

    last_joint_state = msg

### TODO: This block should probably move to a library

def joint(action):
  last_states = last_joint_state

  joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]
  cur_angles = []

  # Reorder the joint names to be the order specified by joint_names
  for name in joint_names:
    cur_angles.append(last_states.position[last_states.name.index(name)])

  # The desired angles from the payload
  new_angles = [float(q) for q in action["payload"]]

  duration = float(action["duration"])
  NUM_PTS = 50

  # Calculate the quintic trajectory for each joint. Impose 0 velocity and 0 acceleration at limits.
  traj_triplets = []
  for (p0, pf) in zip(cur_angles, new_angles):
    traj = TrajectoryPlanner.quintic_interp(action["duration"], p0, pf, 0, 0, 0, 0, NUM_PTS)
    traj_triplets.append(traj)

  traj_pts = []

  # 
  for i in range(NUM_PTS):
    pt = JointTrajectoryPoint()
    pt.positions     = [traj_triplets[joint][0][i] for joint in range(5)]
    pt.velocities    = [traj_triplets[joint][1][i] for joint in range(5)]
    pt.accelerations = [traj_triplets[joint][2][i] for joint in range(5)]

    pt.time_from_start = rospy.Duration(i * (duration / NUM_PTS))

    traj_pts.append(pt)

  trajectory = JointTrajectory()
  trajectory.joint_names = joint_names
  trajectory.points = traj_pts

  trajectory.header.stamp = rospy.Time.now()
  trajectory.header.frame_id = "world"

  trajectory_pub.publish(trajectory)

  rospy.sleep(duration)

def magnet(action):
  state = MagnetState()

  state.magnet1 = action["payload"]["magnet1"]
  state.magnet2 = action["payload"]["magnet2"]

  mag_state_pub.publish(state)
  rospy.sleep(float(action["duration"]))

def nfc(action):
  print("nfc")

def comm(action):
  print("comm")

def delay(action):
  rospy.sleep(float(action["duration"]))

######################################################

def main():
  global trajectory_pub, mag_state_pub
  rospy.init_node("demo_runner")

  rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB)
  trajectory_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)
  mag_state_pub = rospy.Publisher("/inchworm/magnet_states", MagnetState, queue_size=1)

  rospack = rospkg.RosPack()

  demos = glob.glob(rospack.get_path("inchworm_control") + "/demos/*.yaml")

  print(f"Available demos: {' '.join([name.split('/')[-1][:-5] for name in demos])}")
  demo = input("Which demo would you like? Choose from the list above: ")

  path = f"{rospack.get_path('inchworm_control')}/demos/{demo}.yaml"

  data = None

  with open(path, "r") as f:
    try:
      data = yaml.safe_load(f)
    except yaml.YAMLError as e:
      print(e)
      sys.exit()

  print(data)

  actions = list(data.values())[0]

  ACTION_FN_MAP = {"joint": joint, "magnet": magnet, "nfc": nfc, "comm": comm, "delay": delay}

  # Run each action sequentially
  for action in actions:
    data = list(action.values())[0]

    # If the type is in our function map, invoke the function and pass in the action data
    if data["type"] in ACTION_FN_MAP:
      name = list(action.keys())[0]
      print(f"Running action {name}")
      ACTION_FN_MAP[data["type"]](data)
    else:
      rospy.logerr(f"Invalid action type {data['type']}, quitting")
      sys.exit()

if __name__ == "__main__":
  main()