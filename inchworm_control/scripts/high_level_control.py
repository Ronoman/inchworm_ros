#!/usr/bin/env python3

import rospy, time

from inchworm import Inchworm


def main():
  rospy.init_node("high_level_control")

  robot_count = rospy.get_param("robot_count", 1)

  iws = []

  time.sleep(10)

  for i in range(robot_count):
    iw = Inchworm(idx=i)

    iws.append(iw)

  iws[0].move(Inchworm.Neighbors.RIGHT)
  iws[0].move(Inchworm.Neighbors.NONE)

  rospy.spin()

if __name__ == "__main__":
  main()