#!/usr/bin/env python

from numpy import array, matrix, arccos, sqrt, clip, zeros
from numpy.linalg import norm
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_conjugate
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_multiply
from tf2_ros import TransformStamped

def saturate(v, m):
  norm_v = norm(v)
  if norm_v > m:
    return m * v/norm_v
  return v

def saturate_infinite_norm(v, m):
  max_v = max(v)
  if max_v > m:
    return list(m/max_v * array(v))
  return v

def rotation_matrix(q):
  return matrix(quaternion_matrix(q)[:3,:3])

def skew(v):
  return matrix([[0.0, -v[2,0], v[1,0]],
                 [v[2,0], 0.0, -v[0,0]],
                 [-v[1,0], v[0,0], 0.0]])

def pq_multiply(pBA, qBA, pCB, qCB):
  pCA = pBA + rotation_matrix(qBA) * pCB
  qCA = quaternion_multiply(qBA, qCB)
  return (pCA, qCA)

def pq_inverse(pBA, qBA):
  qAB = quaternion_conjugate(qBA)
  pAB = -rotation_matrix(qAB) * pBA
  return (pAB, qAB)

def pq_divide(pBA, qBA, pCA, qCA):
  qAB = quaternion_conjugate(qBA)
  qCB = quaternion_multiply(qAB, qCA)
  pCB = rotation_matrix(qAB) * (pCA - pBA)
  return (pCB, qCB)

def tf2T(transform):
  return (matrix([[transform.transform.translation.x],
              [transform.transform.translation.y],
              [transform.transform.translation.z]]),
          array([transform.transform.rotation.x,
                 transform.transform.rotation.y,
                 transform.transform.rotation.z,
                 transform.transform.rotation.w]))

def T2tf(T, transform):
  transform.transform.translation.x = T[0][0]
  transform.transform.translation.y = T[0][1]
  transform.transform.translation.z = T[0][2]
  transform.transform.rotation.x = T[1][0]
  transform.transform.rotation.y = T[1][1]
  transform.transform.rotation.z = T[1][2]
  transform.transform.rotation.w = T[1][3]

def q2r(q):
  if q[3] < 0.0:
    q *= -1.0
  if q[3] < 1.0 - 1.0e-6:
    return (2.0 * arccos(clip(q[3], -1.0, 1.0)))/sqrt(1.0 - q[3]**2) * q[:3]
  else:
    return zeros(3)

def pose2T(pose):
  return [[pose.pose.position.x,
           pose.pose.position.y,
           pose.pose.position.z],
          [pose.pose.orientation.x,
           pose.pose.orientation.y,
           pose.pose.orientation.z,
           pose.pose.orientation.w]]

