import threading
import time

import numpy as np

import rospy

from webots_ros.srv import get_bool, get_boolRequest, get_boolResponse
from webots_ros.srv import get_float, get_floatRequest, get_floatResponse
from webots_ros.srv import set_float, set_floatRequest, set_floatResponse
from controller_manager_msgs.srv import LoadController, LoadControllerRequest, LoadControllerResponse, UnloadController, UnloadControllerRequest, UnloadControllerResponse, SwitchController, SwitchControllerRequest, SwitchControllerResponse

import dearpygui.dearpygui as dpg


class WebotsDashboardWithROSControl:
    def __init__(self, supervisor_srv = '/supervisor', controller_mgr_srv = '/controller_manager', controller_name = 'controller'):
        # private
        self.ready = threading.Event()
        
        # public
        self.supervisor_srv = supervisor_srv
        self.controller_mgr_srv = controller_mgr_srv
        self.controller_name = controller_name
        
        # services
        SERVICE_SIMULATION_RESET = self.supervisor_srv + '/simulation_reset'
        SERVICE_UNLOAD_CONTROLLER = self.controller_mgr_srv + '/unload_controller'
        SERVICE_LOAD_CONTROLLER = self.controller_mgr_srv + '/load_controller'
        SERVICE_SWITCH_CONTROLLER = self.controller_mgr_srv + '/switch_controller'
        rospy.wait_for_service(SERVICE_SIMULATION_RESET)
        rospy.wait_for_service(SERVICE_UNLOAD_CONTROLLER)
        rospy.wait_for_service(SERVICE_LOAD_CONTROLLER)
        rospy.wait_for_service(SERVICE_SWITCH_CONTROLLER)
        self.srv_simulation_reset = rospy.ServiceProxy(SERVICE_SIMULATION_RESET, get_bool)
        self.srv_unload_controller = rospy.ServiceProxy(SERVICE_UNLOAD_CONTROLLER, UnloadController)
        self.srv_load_controller = rospy.ServiceProxy(SERVICE_LOAD_CONTROLLER, LoadController)
        self.srv_switch_controller = rospy.ServiceProxy(SERVICE_SWITCH_CONTROLLER, SwitchController)
        
        # ready
        self.ready.set()
    
    def is_ready(self):
        return self.ready.is_set()
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for simulation dashboard ...')
            time.sleep(0.2)
        rospy.loginfo('Simulation dashboard is ready.')
    
    def reset_simulation(self):
        try:
            request = get_boolRequest(1)
            response = self.srv_simulation_reset(request)
        except rospy.ServiceException as e:
            rospy.logerr('Reset simulation failed: %s' % e)

    def unload_controller(self):
        try:
            request = UnloadControllerRequest(self.controller_name)
            response = self.srv_unload_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Unload controller failed: %s' % e)

    def load_controller(self):
        try:
            request = LoadControllerRequest(self.controller_name)
            response = self.srv_load_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Load controller failed: %s' % e)
    
    def stop_controller(self):
        try:
            request = SwitchControllerRequest(stop_controllers = [self.controller_name])
            response = self.srv_switch_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Stop controller failed: %s' % e)

    def start_controller(self):
        try:
            request = SwitchControllerRequest(start_controllers = [self.controller_name])
            response = self.srv_switch_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Start controller failed: %s' % e)
    
    def reset_all(self):
        self.reset_simulation()
        self.stop_controller()
        self.unload_controller()
        self.load_controller()
        self.start_controller()


class WebotsRotationalMotorController:
    INFINITY = float('inf')

    def __init__(self, motor_name, namespace = ''):
        self.SERVICE_SET_POSITION = namespace + '/' + motor_name + '/set_position'
        self.SERVICE_SET_VELOCITY = namespace + '/' + motor_name + '/set_velocity'
        self.SERVICE_SET_TORQUE = namespace + '/' + motor_name + '/set_torque'
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


class WebotsAckermannController:
    def __init__(self,
        left_front_steer_motor_name, # = 'left_front_steer_motor',
        right_front_steer_motor_name, # = 'right_front_steer_motor',
        left_rear_motor_name, # = 'left_rear_motor',
        right_rear_motor_name, # = 'right_rear_motor',
        l, # = 1.628, # 轮间距
        d, # = 2.995, # 前后轴距
        r, # = 0.38, # 轮半径
        max_phi, # 0.78, # 单轮最大转角
        namespace = '' # = '/car'
    ):
        # parameters
        self.l = l
        self.d = d
        self.r = r
        self.max_phi = max_phi

        # motors
        self.left_front_steer_motor = WebotsRotationalMotorController(left_front_steer_motor_name, namespace)
        self.right_front_steer_motor = WebotsRotationalMotorController(right_front_steer_motor_name, namespace)
        self.left_rear_motor = WebotsRotationalMotorController(left_rear_motor_name, namespace)
        self.right_rear_motor = WebotsRotationalMotorController(right_rear_motor_name, namespace)

        # services
        self.SERVICE_ROBOT_GET_TIME = namespace + '/robot/get_time'
        rospy.wait_for_service(self.SERVICE_ROBOT_GET_TIME)
        self.srv_robot_get_time = rospy.ServiceProxy(self.SERVICE_ROBOT_GET_TIME, get_float)
    
    def get_time(self):
        try:
            request = get_floatRequest(0)
            response = self.srv_robot_get_time(request)
            return response.value
        except rospy.ServiceException as e:
            rospy.logerr('Get time failed: %s' % e)

    def command(self, v, w, try_best_w): # return v, R
        sgn = np.sign(v + 1e-3) * np.sign(w + 1e-3)
        
        R_min = sgn * (self.d / np.tan(self.max_phi) + self.l / 2)
        R = float('inf') if abs(w) < 1e-3 else v / w

        if abs(R) < abs(R_min):
            if not try_best_w:
                rospy.logerr('Invalid command: v = %f, w = %f' % (v, w))
                return 0, R_min
            R = R_min
        
        phi_l = np.arctan(self.d / (R + self.l / 2))
        phi_r = np.arctan(self.d / (R - self.l / 2))

        w_rear = v / self.r

        self.left_front_steer_motor.set_position(phi_l)
        self.right_front_steer_motor.set_position(phi_r)
        self.left_rear_motor.set_velocity(w_rear)
        self.right_rear_motor.set_velocity(w_rear)

        return v, R

