#! /usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from numpy import array, sin, cos, tan, arccos, clip, pi, sqrt, abs
from numpy.linalg import norm
from rospy import logdebug, loginfo, logwarn, logfatal
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimIiwa7DemoNode(object):
  def __init__(self):
    rospy.init_node('sim_iiwa7_servo_ik', log_level = rospy.INFO)
    rate = rospy.Rate(100)
    cartesian_pose_pub = rospy.Publisher('command/pose', PoseStamped, queue_size = 1)

    while not rospy.is_shutdown():
      t = rospy.Time.now().to_sec()
      cartesian_pose = PoseStamped()
      cartesian_pose.pose.position.x = 0.5 + 0.1 * cos(2 * pi * 0.25 * t)
      cartesian_pose.pose.position.y = 0.1 * sin(2 * pi * 0.25 * t)
      cartesian_pose.pose.position.z = 0.4
      cartesian_pose.pose.orientation.x = 0.0
      cartesian_pose.pose.orientation.y = 1.0
      cartesian_pose.pose.orientation.z = 0.0
      cartesian_pose.pose.orientation.w = 0.0
      cartesian_pose_pub.publish(cartesian_pose)
      rate.sleep()

if __name__ == '__main__':
  sim_iiwa7_demo_node = SimIiwa7DemoNode()
