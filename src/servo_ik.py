#! /usr/bin/env python

import kdl_parser_py.urdf as URDF
import srdfdom.srdf as SRDF
import numpy as np
import PyKDL as kdl
import rospy
import tf

from numpy import pi, array
from rospy import logdebug
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion, quaternion_from_euler
from utilities import saturate_infinite_norm, F2T

SATURATION = 0.1
MAX_ITERATIONS = 100
LINEAR_TOLERANCE = 1.0e-6
ANGULAR_TOLERANCE = 1.0e-5

class ServoIk(object):
  """
  IIWA Kinematics with PyKDL
  """
  def __init__(self, urdf, srdf):
    self._srdf = SRDF.Group.from_xml_string(srdf)
    self._urdf = URDF.urdf.URDF.from_xml_string(urdf)
    self._tree = URDF.treeFromUrdfModel(self._urdf)[1]
    self.base_link = self._srdf.groups[0].chains[0].base_link
    self.tip_link = self._srdf.groups[0].chains[0].tip_link
    self._tip_frame = kdl.Frame()
    self._chain = self._tree.getChain(self.base_link, self.tip_link)
    self._joint_names = self._urdf.joint_map.keys()
    self._num_jnts = self._chain.getNrOfJoints()

    # Joint limits
    self.lower_limits = []
    self.upper_limits = []
    for jnt in self._urdf.joints:
      if jnt.type != 'fixed':
        self.lower_limits.append(jnt.limit.lower)
        self.upper_limits.append(jnt.limit.upper)

    # KDL Solvers
    self._fk_p = kdl.ChainFkSolverPos_recursive(self._chain)
    self._ik_v = kdl.ChainIkSolverVel_pinv(self._chain)

  def forward(self, t):
    if len(t) != self._num_jnts:
      return None
    else:
      joints_array = kdl.JntArray(self._num_jnts)
      for i in xrange(self._num_jnts):
        joints_array[i] = t[i]
      FEB = kdl.Frame()
      self._fk_p.JntToCart(joints_array, FEB)
      return F2T(FEB)

  def inverse(self, t, TGB):
    if len(t) != self._num_jnts:
      return None
    else:
      joints_array = kdl.JntArray(self._num_jnts)
      for i in xrange(self._num_jnts):
        joints_array[i] = t[i]
      joint_velocities_array = kdl.JntArray(self._num_jnts)
      FEB = kdl.Frame()
      FGB = kdl.Frame(
          kdl.Rotation.Quaternion(TGB[1][0], TGB[1][1], TGB[1][2], TGB[1][3]),
          kdl.Vector(TGB[0][0], TGB[0][1], TGB[0][2]))

      self._fk_p.JntToCart(joints_array, FEB)
      FEGB = FGB * FEB.Inverse()

      iterations = 0
      while (FEGB.p.Norm() > LINEAR_TOLERANCE or FEGB.M.GetRotAngle()[0] > ANGULAR_TOLERANCE) \
            and iterations < MAX_ITERATIONS:
        iterations += 1

        twist = kdl.Twist(FEGB.p, FEGB.M.GetRot())
        self._ik_v.CartToJnt(joints_array, twist, joint_velocities_array)

        maximum = 0.0
        for i in xrange(self._num_jnts):
          if joint_velocities_array[i] > maximum:
            maximum = joint_velocities_array[i]
        if maximum > SATURATION:
          reduction = SATURATION/maximum
        else:
          reduction = 1.0

        for i in xrange(self._num_jnts):
          joints_array[i] += reduction * joint_velocities_array[i]
        self._fk_p.JntToCart(joints_array, FEB)
        FEGB = FGB * FEB.Inverse()

      logdebug('amount of iterations needed: %s', iterations)

      if iterations >= MAX_ITERATIONS:
        return None

      solution = []
      for i in xrange(self._num_jnts):
        while joints_array[i] < pi:
          joints_array[i] += 2.0 * pi
        while joints_array[i] > pi:
          joints_array[i] -= 2.0 * pi
        if joints_array[i] < self.lower_limits[i] \
           or joints_array[i] > self.upper_limits[i]:
          return None
        solution.append(joints_array[i])
      return solution

  def print_robot_description(self):
      nf_joints = 0
      for j in self._urdf.joints:
          if j.type != 'fixed':
              nf_joints += 1
      print "URDF non-fixed joints: %d;" % nf_joints
      print "URDF total joints: %d" % len(self._urdf.joints)
      print "URDF links: %d" % len(self._urdf.links)
      print "KDL joints: %d" % self._tree.getNrOfJoints()
      print "KDL segments: %d" % self._tree.getNrOfSegments()
      print "lower limits: " + str(self.lower_limits_jnt_array)
      print "upper limits: " + str(self.upper_limits_jnt_array)

  def print_kdl_chain(self):
      for i in xrange(self._chain.getNrOfSegments()):
          print '* ' + self._chain.getSegment(i).getName()

if __name__ == '__main__':
  servo_ik = ServoIk()
  q = quaternion_from_euler(0.0, pi - 0.1, 0.0)
  print ik.solve([0.0, 0.0, 0.0, -pi/2.0, 0.0, pi/2.0, 0.0], [[0.3, 0.0, 0.5], q])
