#! /usr/bin/env python

import actionlib
import geometry_msgs.msg as geometry_msgs
import rospy
import std_msgs.msg as std_msgs
import tf2_ros
import yaml

from geometry_msgs.msg import PoseStamped
from servo_ik_module import ServoIk
from numpy import array, sin, cos, tan, arccos, clip, pi, sqrt, abs
from numpy.linalg import norm
from rospy import logdebug, loginfo, logwarn, logwarn_throttle, logfatal
from sensor_msgs.msg import JointState
from servo_ik.srv import SetSpeed, SetSpeedRequest, SetSpeedResponse
from std_msgs.msg import Header
from utilities import tf2T, T2tf, pose2T, T2pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ServoIkNode(object):
  def __init__(self):
    rospy.init_node('servo_ik', log_level = rospy.INFO)
    self.kinematics = ServoIk(
        rospy.get_param('/robot_description'),
        rospy.get_param('/robot_description_semantic'),
        rospy.get_param('~group_name'))

    self.joint_names = None
    self.speed = 0.5

    self.set_speed = rospy.Service('set_speed', SetSpeed, self.handleSetSpeed)
    self.joint_trajectory_pub = rospy.Publisher(
        'PositionJointInterface_trajectory_controller/command', JointTrajectory, queue_size = 1)
    self.state_pose_pub = rospy.Publisher( 'state/pose', PoseStamped, queue_size = 1)
    self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.jointStateCb, queue_size = 1)#rospy.get_namespace()+
    self.command_pose_sub = rospy.Subscriber('command/pose', PoseStamped, self.cartesianPoseCb, queue_size = 1)
    self.state_wrench_pub = rospy.Publisher('state/wrench', geometry_msgs.WrenchStamped, queue_size = 1)

    rospy.spin()

  def handleSetSpeed(self, request):
    self.speed = request.speed
    return SetSpeedResponse(True)

  def publishFakeWrench(self):
    self.state_wrench_pub.publish(
        geometry_msgs.WrenchStamped(
            header = std_msgs.Header(frame_id = self.kinematics.tip_link),
            wrench = geometry_msgs.Wrench(
              force = geometry_msgs.Vector3(
                x = 0.0, y = 0.0, z = 0.0))))

  def jointStateCb(self, msg):
    self.joint_names = msg.name
    self.position = msg.position
    ik = self.kinematics.forward(self.position)
    if ik:
        self.state_pose_pub.publish(T2pose(ik, self.kinematics.base_link))
        self.publishFakeWrench()

  def cartesianPoseCb(self, msg):
    if self.joint_names != None:
      t = self.kinematics.inverse(self.position, pose2T(msg))
      #t = self.kinematics.inverse([0.0, 0.0, 0.0, -pi/2.0, 0.0, pi/2.0, 0.0], pose2T(msg))
      if t != None:
        jtp = JointTrajectoryPoint()
        jtp.positions = t
        jtp.time_from_start = rospy.Duration.from_sec(self.speed)
        jt = JointTrajectory()
        jt.joint_names = self.joint_names[0:self.kinematics._num_jnts]
        jt.points.append(jtp)
        self.joint_trajectory_pub.publish(jt)
      else:
        logwarn_throttle(1.0, '[ServoIk] no inverse kinematics solution found')
    else:
      logwarn_throttle(1.0, 'position not initialized')

if __name__ == '__main__':
  servo_ik_node = ServoIkNode()
