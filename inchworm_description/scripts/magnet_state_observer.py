#!/usr/bin/env python3

from contextlib import suppress
import rospy

from inchworm_hw_interface.msg import MagnetState
from assembly_msgs.srv import SetMateSuppression, SetMateSuppressionRequest

last_mag_state = MagnetState(magnet1=True, magnet2=True)
suppress_proxy = None

def inchwormMagCB(msg):
  global last_mag_state

  suppressions = []

  # If either state has changed, we need to update suppression. A mate is suppressed if the magnet is off.
  # TOOD: Generalize to multiple robots
  if msg.magnet1 != last_mag_state.magnet1:
    # Compose the scoped name of the link to disable mates for
    link = ["inchworm", "inchworm_description_0", "iw_root_0"]
    suppress = (link, not msg.magnet1)
    suppressions.append(suppress)
  if msg.magnet2 != last_mag_state.magnet2:
    link = ["inchworm", "inchworm_description_0", "iw_foot_top_0"]
    suppress = (link, not msg.magnet2)
    suppressions.append(suppress)

  for suppression in suppressions:
    # Compose a SetMateSuppression request
    link = suppression[0]
    suppress = suppression[1]

    req = SetMateSuppressionRequest()
    req.scoped_link = link
    req.suppress = suppress

    rospy.loginfo(f"Setting link {'::'.join(link)} suppression to {suppress}")
    # Ensure that the service returns True, otherwise error.
    res = suppress_proxy(req)

    # If it should be suppressed but it wasn't, log an error
    if suppress and not res.suppressed:
      rospy.logerr(f"Link {'::'.join(link)} failed to suppress.")

  last_mag_state = msg

def main():
  global suppress_proxy
  '''
  This node reads in state from the inchworm, and controls magnet suppression in the magnet sim.
  TODO: This script assumes one inchworm. Update to get robot count from param server and iterate over all.
  '''

  rospy.init_node("magnet_state_observer")
  
  rospy.loginfo("Waiting for /suppress_mate service...")
  rospy.wait_for_service("/suppress_mate")
  rospy.loginfo("Found service.")
  suppress_proxy = rospy.ServiceProxy("/suppress_mate", SetMateSuppression)

  rospy.Subscriber("/inchworm/magnet_states", MagnetState, inchwormMagCB)

  rospy.spin()

if __name__ == "__main__":
  main()