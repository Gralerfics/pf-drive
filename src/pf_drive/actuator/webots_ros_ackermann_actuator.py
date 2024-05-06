import time
import multiprocessing as mp

import numpy as np

import transforms3d as t3d

from std_msgs.msg import Float64
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_bool, get_boolRequest, get_boolResponse
from webots_ros.srv import get_float, get_floatRequest, get_floatResponse
from webots_ros.srv import set_float, set_floatRequest, set_floatResponse

from multinodes import Node

from pf_drive.util import ROSContext, t3d_ext


class WebotsRotationalMotorController:
    INFINITY = float('inf')

    def __init__(self, motor_srv, ros_context):
        self.set_position_srv = motor_srv + '/set_position'
        self.set_velocity_srv = motor_srv + '/set_velocity'
        self.set_torque_srv = motor_srv + '/set_torque'

        self.ros_context = ros_context
        self.ros_context.register_service(self.set_position_srv, set_float)
        self.ros_context.register_service(self.set_velocity_srv, set_float)
        self.ros_context.register_service(self.set_torque_srv, set_float)
    
    def set_position(self, position):
        response = self.ros_context.call_service(self.set_position_srv, set_floatRequest(value = position))
        return response.success if response is not None else False

    def set_velocity(self, velocity):
        if not self.set_position(self.INFINITY):
            return False
        response = self.ros_context.call_service(self.set_velocity_srv, set_floatRequest(value = velocity))
        return response.success if response is not None else False
    
    def set_torque(self, torque):
        response = self.ros_context.call_service(self.set_torque_srv, set_floatRequest(value = torque))
        return response.success if response is not None else False


"""
    `command`, input (pipe)
        format:
            (type = 'vw', v, w)
            (type = 'vphi', v, phi)
    `odom`, output (any)
        format:
            4x4 np.array
    `param`, output (any)
        format:
            [phi_l, phi_r, w_rear]
"""
class WebotsROSAckermannActuatorComputer(Node):
    def __init__(self, name,
        get_time_srv,
        track,
        wheelbase,
        wheel_radius,
        max_steering_angle,
        left_rear_position_topic,
        right_rear_position_topic
    ):
        super().__init__(name)

        self.get_time_srv = get_time_srv

        self.l = track
        self.d = wheelbase
        self.r = wheel_radius
        self.max_phi = max_steering_angle

        self.left_rear_position_topic = left_rear_position_topic
        self.right_rear_position_topic = right_rear_position_topic

        self.R_min_abs = self.d / np.tan(self.max_phi) + self.l / 2

        self.odom = np.eye(4)
        self.v_rec = 0
        self.R_rec = 1e9

        self.ros = ROSContext(self.name)
        self.ros.register_service(self.get_time_srv, get_float)

        self.angle_l_last, self.angle_l = None, None
        self.angle_r_last, self.angle_r = None, None

    def get_time(self):
        response = self.ros.call_service(self.get_time_srv, get_floatRequest(0))
        return response.value if response is not None else None

    def left_rear_position_callback(self, msg):
        self.angle_l_last = self.angle_l
        self.angle_l = msg.data
        if self.angle_l_last is None:
            self.angle_l_last = msg.data
    
    def right_rear_position_callback(self, msg):
        self.angle_r_last = self.angle_r
        self.angle_r = msg.data
        if self.angle_r_last is None:
            self.angle_r_last = msg.data
    
    def get_d(self):
        if self.angle_l is None or self.angle_l_last is None or self.angle_r is None or self.angle_r_last is None:
            return 0
        d_l = (self.angle_l - self.angle_l_last) * self.r
        d_r = (self.angle_r - self.angle_r_last) * self.r
        return (d_l + d_r) / 2

    def update_odom(self, d, R):
        dist = d
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
    
    def run(self):
        self.ros.init_node(anonymous = False) # 不可在 __init__ 中调用，否则会导致和 main 中的 init_node 冲突
        self.ros.subscribe_topic(self.left_rear_position_topic, Float64Stamped, self.left_rear_position_callback)
        self.ros.subscribe_topic(self.right_rear_position_topic, Float64Stamped, self.right_rear_position_callback)
        
        current_time = self.get_time()
        last_time = current_time
        while not self.ros.is_shutdown():
            # 处理 ROS 消息回调
            self.ros.spin_once(0.01) # TODO: 不加也能正常订阅, why, 因为有其他进程在 spin?

            # 里程计 (不受 command 影响, 有无 command 都会计算里程计)
            current_time = self.get_time()
            if current_time is None or last_time is None:
                last_time = current_time
                continue
            dt = current_time - last_time
            last_time = current_time

            if 'odom' in self.io.keys():
                self.update_odom(self.get_d(), self.R_rec) # 轮速 + 转向指令
                self.io['odom'].write(self.odom)
            
            # 检查接口
            if 'command' in self.io.keys():
                # 接收指令
                if not self.io['command'].poll():
                    continue
                command = self.io['command'].read()
                if not (isinstance(command, tuple) or isinstance(command, list)):
                    continue
                
                # 计算执行器指令
                w, phi = None, None
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

                self.v_rec = v
                self.R_rec = R

                # 输出参数
                if 'param' in self.io.keys():
                    self.io['param'].write([phi_l, phi_r, w_rear])


"""
    `param`, input (any)
        format:
            [phi_l, phi_r, w_rear]
"""
class WebotsROSAckermannActuatorCaller(Node):
    def __init__(self, name,
        left_front_steer_motor_srv,
        right_front_steer_motor_srv,
        left_rear_motor_srv,
        right_rear_motor_srv
    ):
        super().__init__(name)
        
        self.left_front_steer_motor_srv = left_front_steer_motor_srv
        self.right_front_steer_motor_srv = right_front_steer_motor_srv
        self.left_rear_motor_srv = left_rear_motor_srv
        self.right_rear_motor_srv = right_rear_motor_srv

        self.ros = ROSContext(self.name)
        self.left_front_steer_motor = WebotsRotationalMotorController(self.left_front_steer_motor_srv, self.ros)
        self.right_front_steer_motor = WebotsRotationalMotorController(self.right_front_steer_motor_srv, self.ros)
        self.left_rear_motor = WebotsRotationalMotorController(self.left_rear_motor_srv, self.ros)
        self.right_rear_motor = WebotsRotationalMotorController(self.right_rear_motor_srv, self.ros)
    
    def run(self):
        self.ros.init_node(anonymous = False) # 不可在 __init__ 中调用，否则会导致和 main 中的 init_node 冲突

        while not self.ros.is_shutdown():
            if 'param' not in self.io.keys():
                time.sleep(0.1)
                continue

            if not self.io['param'].poll():
                continue
            phi_l, phi_r, w_rear = self.io['param'].read()

            self.left_front_steer_motor.set_position(phi_l)
            self.right_front_steer_motor.set_position(phi_r)
            self.left_rear_motor.set_velocity(w_rear)
            self.right_rear_motor.set_velocity(w_rear)

