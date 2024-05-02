import time

import numpy as np

import transforms3d as t3d

import rospy

from webots_ros.srv import supervisor_get_from_defRequest, supervisor_get_from_defResponse, supervisor_get_from_def
from webots_ros.srv import node_get_positionRequest, node_get_positionResponse, node_get_position, node_get_orientationRequest, node_get_orientationResponse, node_get_orientation

from multinodes import Node

from pf_drive.util import t3d_ext


"""
    `gt_pose`, output (pipe)
"""
class WebotsROSRobotGlobalLocator(Node):
    def __init__(self, name, is_shutdown_event, robot_def, supervisor_srv):
        super().__init__(name, is_shutdown_event)
        self.robot_def = robot_def
        self.supervisor_srv = supervisor_srv

        SERVICE_GET_FROM_DEF = self.supervisor_srv + '/get_from_def'
        SERVICE_GET_POSITION = self.supervisor_srv + '/node/get_position'
        SERVICE_GET_ORIENTATION = self.supervisor_srv + '/node/get_orientation'
        rospy.wait_for_service(SERVICE_GET_FROM_DEF)
        rospy.wait_for_service(SERVICE_GET_POSITION)
        rospy.wait_for_service(SERVICE_GET_ORIENTATION)
        self.srv_get_from_def = rospy.ServiceProxy(SERVICE_GET_FROM_DEF, supervisor_get_from_def)
        self.srv_get_position = rospy.ServiceProxy(SERVICE_GET_POSITION, node_get_position)
        self.srv_get_orientation = rospy.ServiceProxy(SERVICE_GET_ORIENTATION, node_get_orientation)

        # 获取 robot 节点句柄 (node)
        try:
            request = supervisor_get_from_defRequest(name = self.robot_def, proto = 0)
            response = self.srv_get_from_def(request)
            self.robot_node_handle = response.node # 获取 node 句柄
        except rospy.ServiceException as e:
            rospy.logerr('Service call get_from_def failed.') # TODO
    
    def run(self):
        while not self.is_shutdown() and not rospy.is_shutdown():
            if 'gt_pose' not in self.io:
                time.sleep(0.1)
                continue

            try:
                request = node_get_positionRequest(node = self.robot_node_handle)
                response = self.srv_get_position(request)
                t = np.array([response.position.x, response.position.y, response.position.z])
            except rospy.ServiceException as e:
                rospy.logerr('Service call get_position failed.')
            
            try:
                request = node_get_orientationRequest(node = self.robot_node_handle)
                response = self.srv_get_orientation(request)
                q = np.array([response.orientation.w, response.orientation.x, response.orientation.y, response.orientation.z]) # t3d, wxyz
            except rospy.ServiceException as e:
                rospy.logerr('Service call get_orientation failed.')
            
            T = t3d_ext.etq(t, q)
            self.io['gt_pose'].write(T)

            time.sleep(0.02)

