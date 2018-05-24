#! /usr/bin/env python

import actionlib
import rospy
import tf2_ros
import yaml

from geometry_msgs.msg import PoseStamped
from servo_ik import ServoIk
from numpy import array, sin, cos, tan, arccos, clip, pi, sqrt, abs
from numpy.linalg import norm
from rospy import logdebug, loginfo, logwarn, logfatal
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from utilities import tf2T, T2tf, pq_multiply, pq_inverse, q2r, pose2T
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimIiwa7ServoIkNode(object):
  def __init__(self):
    rospy.init_node('sim_iiwa7_servo_ik', log_level = rospy.INFO)
    self.ik = ServoIk(rospy.get_param('/robot_description'))

    self.joint_names = None

    self.joint_trajectory_pub = rospy.Publisher(
        'PositionJointInterface_trajectory_controller/command', JointTrajectory, queue_size = 1)
    self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.jointStateCb, queue_size = 1)
    self.cartesian_pose_sub = rospy.Subscriber('cartesian_pose', PoseStamped, self.cartesianPoseCb, queue_size = 1)

    rospy.spin()

  def jointStateCb(self, msg):
    self.joint_names = msg.name
    self.position = msg.position

  def cartesianPoseCb(self, msg):
    if self.joint_names != None:
      t = self.ik.solve([0.0, 0.0, 0.0, -pi/2.0, 0.0, pi/2.0, 0.0], pose2T(msg))
      if t != None:
        jtp = JointTrajectoryPoint()
        jtp.positions = t
        jtp.time_from_start = rospy.Duration.from_sec(0.5)
        jt = JointTrajectory()
        jt.joint_names = self.joint_names
        jt.points.append(jtp)
        self.joint_trajectory_pub.publish(jt)
      else:
        logwarn('ik solution not found')
    else:
      logwarn('position not initialized')

if __name__ == '__main__':
  sim_iiwa7_servo_ik_node = SimIiwa7ServoIkNode()
