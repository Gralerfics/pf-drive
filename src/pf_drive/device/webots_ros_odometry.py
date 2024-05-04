import time

import numpy as np

import transforms3d as t3d

from webots_ros.srv import supervisor_get_from_defRequest, supervisor_get_from_def
from webots_ros.srv import node_get_positionRequest, node_get_position, node_get_orientationRequest, node_get_orientation, node_get_velocityRequest, node_get_velocity

from multinodes import Node

from pf_drive.util import t3d_ext
from pf_drive.util import ROSContext


"""
    `gt_pose`, output (pipe) [optional]
        format: 4x4 np.array
    `gt_twist`, output (pipe) [optional]
        format: np.array([vx, vy, vz, wx, wy, wz])
"""
class WebotsROSRobotGlobalLocator(Node):
    def __init__(self, name, robot_def, supervisor_srv):
        super().__init__(name)
        self.robot_def = robot_def
        self.supervisor_srv = supervisor_srv

        self.get_from_def_srv = self.supervisor_srv + '/get_from_def'
        self.get_position_srv = self.supervisor_srv + '/node/get_position'
        self.get_orientation_srv = self.supervisor_srv + '/node/get_orientation'
        self.get_velocity_srv = self.supervisor_srv + '/node/get_velocity'

        self.ros = ROSContext(self.name)
        self.ros.register_service(self.get_from_def_srv, supervisor_get_from_def)
        self.ros.register_service(self.get_position_srv, node_get_position)
        self.ros.register_service(self.get_orientation_srv, node_get_orientation)
        self.ros.register_service(self.get_velocity_srv, node_get_velocity)
    
    def run(self):
        self.ros.init_node(anonymous = False)

        # 尝试获取 robot 节点句柄, 注意在 init_node 之后
        self.robot_node_handle = None
        while self.robot_node_handle is None and not self.ros.is_shutdown():
            response = self.ros.call_service(self.get_from_def_srv, supervisor_get_from_defRequest(name = self.robot_def, proto = 0))
            self.robot_node_handle = response.node if response is not None else None
        
        while not self.ros.is_shutdown():
            if 'gt_pose' in self.io:
                response = self.ros.call_service(self.get_position_srv, node_get_positionRequest(node = self.robot_node_handle))
                if response is None:
                    self.ros.logerr('Service call get_position failed.')
                    time.sleep(0.1)
                    continue
                t = np.array([response.position.x, response.position.y, response.position.z])

                response = self.ros.call_service(self.get_orientation_srv, node_get_orientationRequest(node = self.robot_node_handle))
                if response is None:
                    self.ros.logerr('Service call get_orientation failed.')
                    time.sleep(0.1)
                    continue
                q = np.array([response.orientation.w, response.orientation.x, response.orientation.y, response.orientation.z]) # wxyz in t3d
                
                T = t3d_ext.etq_wxyz(t, q)
                self.io['gt_pose'].write(T)
            
            if 'gt_twist' in self.io:
                response = self.ros.call_service(self.get_velocity_srv, node_get_velocityRequest(node = self.robot_node_handle))
                if response is None:
                    self.ros.logerr('Service call get_velocity failed.')
                    time.sleep(0.1)
                    continue
                v = response.velocity
                res = np.array([v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z])
                self.io['gt_twist'].write(res)
            
            time.sleep(0.02)

