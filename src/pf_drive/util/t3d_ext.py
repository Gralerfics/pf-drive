import math

import numpy as np

import transforms3d as t3d

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion


"""
    直接拼接耗时是
        t3d.affines.compose(t, t3d.quaternions.quat2mat(q), np.ones(3))
    的 0.5 至 1 倍, 故 ext.
"""
def euclidean_compose_tq(t, q):
    res = np.zeros((4, 4))
    res[:3, :3] = t3d.quaternions.quat2mat(q)
    res[:3, 3] = t
    res[3, 3] = 1
    return res

def euclidean_compose_tR(t, R):
    res = np.zeros((4, 4))
    res[:3, :3] = R
    res[:3, 3] = t
    res[3, 3] = 1
    return res

def euclidean_decompose_t(T):
    return T[:3, 3]

def euclidean_decompose_q(T):
    return t3d.quaternions.mat2quat(T[:3, :3])

def euclidean_decompose_R(T):
    return T[:3, :3]

edt = euclidean_decompose_t
edq = euclidean_decompose_q
edR = euclidean_decompose_R


"""
    用
        R 和 t 拼出 T 的逆
    耗时是使用 np.linalg.inv(T) 的 0.8 倍左右, 故 ext.
"""
def euclidean_inv(T):
    res = np.zeros((4, 4))
    res[:3, :3] = T[:3, :3].T
    res[:3, 3] = -res[:3, :3] @ T[:3, 3]
    res[3, 3] = 1
    return res
einv = euclidean_inv


"""
    用
        np.sqrt(np.dot(x, x))
    计算范数耗时是使用 np.linalg.norm(x) 的 0.6 倍左右, 故 ext.
"""
def norm(x):
    return np.sqrt(np.dot(x, x))

"""
    用
        math.atan2
    耗时是使用 np.arctan2 的 0.25 倍左右, 故 ext.
"""
def atan2(y, x):
    return math.atan2(y, x)


"""
    使用
        t3d.euler.euler2mat(0, 0, theta)
    耗时是使用 numpy 写 sin, cos 的 0.7 倍左右, 故不 ext.
"""
pass


"""
    使用
        math.atan2(R[1, 0], R[0, 0])
    耗时是使用 t3d.euler.mat2euler(R)[2] 的 0.15 倍左右, 故 ext.
"""
def R_to_yaw(R):
    return math.atan2(R[1, 0], R[0, 0])
R2yaw = R_to_yaw


"""
    与 ROS Message 等的转换.
"""
def euclidean_to_Odometry(T, frame_id = '', stamp = None):
    msg = Odometry()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.pose.pose = Pose(
        position = Point(*edt(T)),
        orientation = Quaternion(*edq(T))
    )
    return msg

def Odometry_to_euclidean(msg):
    # 注意 t3d 中的 quat 顺序是 (w, x, y, z)
    return euclidean_compose_tq(
        np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]),
        np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
    )

e2O = euclidean_to_Odometry
O2e = Odometry_to_euclidean

