import time
import multiprocessing as mp

import numpy as np

import transforms3d as t3d

import rospy

from webots_ros.srv import get_bool, get_boolRequest, get_boolResponse
from webots_ros.srv import get_float, get_floatRequest, get_floatResponse
from webots_ros.srv import set_float, set_floatRequest, set_floatResponse

from multinodes import Node

from pf_drive.util import t3d_ext


class WebotsRotationalMotorController:
    INFINITY = float('inf')

    def __init__(self, motor_srv):
        # TODO: service reset when resetting simulation. 不过看起来服务句柄不会丢失.
        self.SERVICE_SET_POSITION = motor_srv + '/set_position'
        self.SERVICE_SET_VELOCITY = motor_srv + '/set_velocity'
        self.SERVICE_SET_TORQUE = motor_srv + '/set_torque'
        rospy.wait_for_service(self.SERVICE_SET_POSITION)
        rospy.wait_for_service(self.SERVICE_SET_VELOCITY)
        rospy.wait_for_service(self.SERVICE_SET_TORQUE)
        self.srv_set_position = rospy.ServiceProxy(self.SERVICE_SET_POSITION, set_float)
        self.srv_set_velocity = rospy.ServiceProxy(self.SERVICE_SET_VELOCITY, set_float)
        self.srv_set_torque = rospy.ServiceProxy(self.SERVICE_SET_TORQUE, set_float)
    
    def set_position(self, position):
        try:
            request = set_floatRequest(value = position)
            response = self.srv_set_position(request)
        except rospy.ServiceException as e:
            rospy.logerr('Set position failed: %s' % e)
    
    def set_velocity(self, velocity):
        try:
            self.set_position(self.INFINITY)
            request = set_floatRequest(value = velocity)
            response = self.srv_set_velocity(request)
        except rospy.ServiceException as e:
            rospy.logerr('Set velocity failed: %s' % e)
    
    def set_torque(self, torque):
        try:
            request = set_floatRequest(value = torque)
            response = self.srv_set_torque(request)
        except rospy.ServiceException as e:
            rospy.logerr('Set torque failed: %s' % e)


"""
    `command`, input (pipe)
        format:
            (type = 'vw', v, w)
            (type = 'vphi', v, phi)
    `odom`, output (any) [optional]
        format:
            4x4 np.array, TODO: 是否 .tolist() 减少少量长度? 需与转换耗时比较.
    Notes:
        1. Cable 建议使用 latest = True 的 pipe.
"""
class WebotsROSAckermannActuator(Node):
    def __init__(self, name, is_shutdown_event,
        left_front_steer_motor_srv,
        right_front_steer_motor_srv,
        left_rear_motor_srv,
        right_rear_motor_srv,
        get_time_srv,
        track,
        wheelbase,
        wheel_radius,
        max_steering_angle
    ):
        super().__init__(name, is_shutdown_event)
        
        self.left_front_steer_motor = WebotsRotationalMotorController(left_front_steer_motor_srv)
        self.right_front_steer_motor = WebotsRotationalMotorController(right_front_steer_motor_srv)
        self.left_rear_motor = WebotsRotationalMotorController(left_rear_motor_srv)
        self.right_rear_motor = WebotsRotationalMotorController(right_rear_motor_srv)
        
        self.l = track
        self.d = wheelbase
        self.r = wheel_radius
        self.max_phi = max_steering_angle

        self.R_min_abs = self.d / np.tan(self.max_phi) + self.l / 2

        rospy.wait_for_service(get_time_srv)
        self.srv_robot_get_time = rospy.ServiceProxy(get_time_srv, get_float)

        self.odom = np.eye(4)

    def get_time(self):
        try:
            request = get_floatRequest(0)
            response = self.srv_robot_get_time(request)
            return response.value
        except rospy.ServiceException as e:
            rospy.logerr('Get time failed: %s' % e)
    
    def call_services(self, phi_l, phi_r, w_rear):
        self.left_front_steer_motor.set_position(phi_l)
        self.right_front_steer_motor.set_position(phi_r)
        self.left_rear_motor.set_velocity(w_rear)
        self.right_rear_motor.set_velocity(w_rear)
    
    def calc_and_output_odom(self, v, R, dt):
        dist = v * dt
        odom_R = t3d_ext.edR(self.odom)

        if R > 1e9: # inf
            self.odom[:3, 3] += np.dot(odom_R, np.array([dist, 0, 0]))
        else:
            theta = dist / R
            rot = t3d.euler.euler2mat(0, 0, theta)

            # P: robot, O: origin, R: instantaneous center of rotation, T: target of P
            PR = np.dot(odom_R, np.array([0, R, 0]))
            RT = -np.dot(rot, PR)

            self.odom[:3, :3] = np.dot(rot, odom_R)
            self.odom[:3, 3] += PR + RT
        
        self.io['odom'].write(self.odom) # 外部已经确保 odom 接口存在
    
    def run(self):
        current_time = self.get_time()
        last_time = current_time
        while not self.is_shutdown() and not rospy.is_shutdown():
            # 检查接口
            if 'command' not in self.io.keys():
                time.sleep(0.1)
                continue

            # 接收指令
            if not self.io['command'].poll():
                continue
            command = self.io['command'].read()
            if not (isinstance(command, tuple) or isinstance(command, list)):
                continue

            # 计算时间
            current_time = self.get_time()
            if current_time is None or last_time is None:
                last_time = current_time
                continue
            dt = current_time - last_time
            last_time = current_time
            
            # 计算执行器指令
            if command[0] == 'vw':
                v, w = command[1], command[2]
                sgn = np.sign(v + 1e-3) * np.sign(w + 1e-3)
                R_min = sgn * self.R_min_abs
                R = float('inf') if abs(w) < 1e-3 else v / w
            elif command[0] == 'vphi':
                v, phi = command[1], command[2]
                sgn = np.sign(v + 1e-3) * np.sign(phi + 1e-3)
                R_min = sgn * self.R_min_abs
                R = float('inf') if abs(phi) < 1e-3 else self.d / np.tan(phi)
            
            if abs(R) < abs(R_min): # 无法实现该转角，使用最大转角
                R = R_min
            
            phi_l = np.arctan(self.d / (R + self.l / 2))
            phi_r = np.arctan(self.d / (R - self.l / 2))
            w_rear = v / self.r

            p = mp.Process(target = self.call_services, args = (phi_l, phi_r, w_rear))
            p.start()

            # 计算里程计
            if 'odom' in self.io.keys():
                # mp.Process(target = self.calc_and_output_odom, args = (v, R, dt)).start()
                self.calc_and_output_odom(v, R, dt)
        
        p.join()

