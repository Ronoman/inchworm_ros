#!/usr/bin/env python3

import rospy, roslaunch
from rospkg import RosPack
from matplotlib import pyplot as plt

from std_msgs.msg import Int32


# Inchworm counts to test. Currently the roof is [20, 20]
INCHWORM_COUNTS = [i+1 for i in range(10)]

WIDTH = 20
HEIGHT = 20
RATE = 500

durations = []

def tickCB(msg):
  global durations

  durations.append(msg.data)

def main():
  global durations
  rospy.init_node("test_algo_sim")

  tick_sub = rospy.Subscriber("/algo/ticks_elapsed", Int32, tickCB)

  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  
  rospack = RosPack()
  algo_path = rospack.get_path("inchworm_algo")

  launch_path = [algo_path + "/launch/algo_sim.launch"]

  for count in INCHWORM_COUNTS:
    print(f"Running test for inchworm count of {count}")
    cli_args = [f"roof_width:={WIDTH}", f"roof_height:={HEIGHT}", f"rate:={RATE}", f"use_gui:=False", f"inchworm_count:={count}"]

    launch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_path + cli_args)[0], cli_args)]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

    launch.start()
    launch.spin()

    # Wait an excessive amount of time to ensure we receive ticks_elapsed
    rospy.sleep(1)

    print(f"Test took {durations[-1]} ticks to finish.")

  plt.scatter(INCHWORM_COUNTS, durations)
  plt.xlabel("Inchworm count")
  plt.ylabel("Total ticks elapsed")
  plt.title("Time to shingle a 10x10 roof")

  plt.show()

if __name__ == "__main__":
  main()