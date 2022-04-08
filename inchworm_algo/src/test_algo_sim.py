#!/usr/bin/env python3

from operator import delitem
import rospy, roslaunch
from rospkg import RosPack
from matplotlib import pyplot as plt
import sys
import csv

from std_msgs.msg import Int32MultiArray





durations = []

def tickCB(msg):
  global durations

  durations.append(msg.data)

def main():
  global durations
  rospy.init_node("test_algo_sim")

  WIDTH = 10
  HEIGHT = 10
  RATE = 5000



  pattern = int(sys.argv[1])
  WIDTH = int(sys.argv[2])
  HEIGHT = int(sys.argv[3])

  INCHWORM_COUNTS = [i+1 for i in range(int(WIDTH/2))]

  tick_sub = rospy.Subscriber("/algo/ticks_elapsed", Int32MultiArray, tickCB)

  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  
  rospack = RosPack()
  algo_path = rospack.get_path("inchworm_algo")

  launch_path = [algo_path + "/launch/algo_sim.launch"]

  for count in INCHWORM_COUNTS:
    print(f"Running test for inchworm count of {count}")
    cli_args = [f"roof_width:={WIDTH}", f"roof_height:={HEIGHT}", f"rate:={RATE}", f"use_gui:=False", f"inchworm_count:={count}", f"name_space:={WIDTH}x{HEIGHT}_{pattern}_{count}"]

    launch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_path + cli_args)[0], cli_args)]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

    launch.start()
    

    # Wait an excessive amount of time to ensure we receive ticks_elapsed

  while len(durations) != int(int(WIDTH)/2):
    rospy.sleep(1)
    rospy.loginfo(len(durations))
  file = open(f"{algo_path}/data/{WIDTH}x{HEIGHT}_pattern{pattern}.csv", "w+")
  writer = csv.writer(file)
  writer.writerows(durations)
  x = []
  y = []
  for element in durations:
    x.append(element[0])
    y.append(element[1])

  plt.scatter(x, y)
  plt.xlabel("Inchworm count")
  plt.ylabel("Total ticks elapsed")
  plt.title(f"Time to shingle a {WIDTH}x{HEIGHT} roof")
  plt.yscale("log")
  # Uncomment if you want to add a limit, Need to know a good top value 
  # ax = plt.gca()
  # ax.set_ylim(1,45000)

  plt.show()

if __name__ == "__main__":
  main()