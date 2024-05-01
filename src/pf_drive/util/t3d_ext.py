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
    使用
        t3d.euler.euler2mat(0, 0, theta)
    耗时是使用 numpy 写 sin, cos 的 0.7 倍左右, 故不 ext.
"""

"""
    与 ROS Message 的转换.
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

