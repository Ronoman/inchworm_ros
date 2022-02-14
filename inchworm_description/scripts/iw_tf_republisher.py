#!/usr/bin/env python3
import rospy

# Because of transformations
import tf_conversions

from tf2_ros import TransformBroadcaster
from  geometry_msgs.msg import TransformStamped

from gazebo_msgs.srv import GetModelState, GetModelStateRequest


def main():
  rospy.init_node('iw_tf_republisher')
  
  rospy.wait_for_service("/gazebo/get_model_state")
  model_state_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

  broadcaster = TransformBroadcaster()

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    # TODO: Generalize to multi-inchworm
    req = GetModelStateRequest(model_name="inchworm::inchworm_description_0", relative_entity_name="world")

    res = model_state_service(req)

    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "world"
    tf.child_frame_id = "iw_root"

    tf.transform.translation = res.pose.position
    tf.transform.rotation    = res.pose.orientation

    broadcaster.sendTransform(tf)

    rate.sleep()

if __name__ == '__main__':
  main()