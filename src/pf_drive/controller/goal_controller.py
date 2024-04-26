import time
import math
import threading

import numpy as np

import rospy
from geometry_msgs.msg import Twist

from pf_drive.util.debug import Debugger
from pf_drive.util.geometry import Frame

from pf_drive.sensor.odometry import Odom


"""
    TODO: 按 Odom 中注明的规范进行修改.
    
    goal 导向的控制器, 可暂停和恢复, 调用注册的回调函数.
        goal 的参考系应为 biased_odom.
    
    set_odometry():
        注册 odometry.
    
    register_x_hook():
        注册新的回调函数到列表.
    
    is_ready():
        为 True 时方允许: 获取和设置 goal; ...
        要求: odometry 已注册且已 ready.
    
    (de)activate():
        若未 activate 则发布零速.
    
    get_x(), set_x():
        线程安全地获取成员.
"""
class GoalController:
    def __init__(self,
        cmd_vel_topic: str,
        k_rho: float, # k_rho > 0
        k_alpha: float, # k_alpha > k_rho
        k_beta: float, # k beta < 0
        k_theta: float,
        velocity_min: float,
        velocity_max: float,
        omega_min: float,
        omega_max: float,
        translation_tolerance: float,
        rotation_tolerance: float
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'goal_controller_debugger')
        self.goal_reached_hooks: list = []
        
        # public
        self.goal: Frame = None
        self.goal_lock = threading.Lock()
        self.activated = False
        
        # parameters
        self.cmd_vel_topic = cmd_vel_topic
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
        self.k_theta = k_theta
        self.velocity_min = velocity_min
        self.velocity_max = velocity_max
        self.omega_min = omega_min
        self.omega_max = omega_max
        self.translation_tolerance = translation_tolerance
        self.rotation_tolerance = rotation_tolerance
        
        # devices
        self.odometry: Odom = None
        
        # topics
        self.init_topics()
    
    def init_topics(self):
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 1)
    
    def set_odometry(self, odometry: Odom):
        self.odometry = odometry
        self.odometry.register_odom_received_hook(self.odom_received)
    
    def register_goal_reached_hook(self, hook):
        self.goal_reached_hooks.append(hook)
    
    def is_ready(self):
        return self.odometry is not None and self.odometry.is_ready()
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for goal controller ...')
            time.sleep(0.1)
        rospy.loginfo('Goal controller is ready.')

    def modify_cmd_vel_topic(self, topic):
        if not self.is_ready():
            return False
        
        self.deactivate()
        self.cmd_vel_topic = topic
        self.pub_cmd_vel.unregister()
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 1)
        self.activate()
        return True

    def set_goal(self, goal: Frame):
        if not self.is_ready():
            return False
        
        with self.goal_lock:
            self.goal = goal
            if goal is not None:
                frame_id = self.odometry.get_biased_odom_frame_id()
                self.debugger.publish('/goal', goal.to_PoseStamped(frame_id = frame_id))
        return True
    
    def get_goal(self):
        if not self.is_ready():
            return False
        
        with self.goal_lock:
            return self.goal
    
    def activate(self):
        if not self.is_ready() or self.activated:
            return False
        
        self.activated = True
        return True
    
    def deactivate(self):
        if not self.is_ready() or not self.activated:
            return False
        
        self.activated = False
        return True
    
    def warp_to_pi(self, x):
        return ((x + math.pi) % (2 * math.pi)) - math.pi # TODO
    
    def calculate(self, dx, dy, theta, goal_theta):
        rho = math.sqrt(dx * dx + dy * dy)
        alpha = self.warp_to_pi(math.atan2(dy, dx) - theta)
        beta = self.warp_to_pi(-theta - alpha + goal_theta)
        return rho, alpha, beta
    
    def scale_velocities(self, _v, _omega):
        v = np.clip(_v, math.copysign(self.velocity_min, _v), math.copysign(self.velocity_max, _v))
        omega = np.clip(_omega, math.copysign(self.omega_min, _omega), math.copysign(self.omega_max, _omega))
        
        if abs(v) > 1e-6:
            turn_rate = abs(omega / v)
            if turn_rate > self.omega_max / self.velocity_max:
                omega = math.copysign(self.omega_max, omega)
                v = self.omega_max / turn_rate
            else:
                omega = math.copysign(self.velocity_max * turn_rate, omega)
                v = self.velocity_max
        
        return v, omega
    
    def odom_received(self, **args):
        if not self.is_ready() or self.get_goal() is None:
            return
        
        if self.activated:
            goal: Frame = self.get_goal()
            odom: Frame = self.odometry.get_biased_odom()
            
            goal_theta = goal.q.Euler[2]
            odom_theta = odom.q.Euler[2]
            delta_t = goal.t - odom.t
            rho, alpha, beta = self.calculate(delta_t.x, delta_t.y, odom_theta, goal_theta)
            
            if rho < self.translation_tolerance or alpha > math.pi / 2 and alpha < -math.pi / 2:
                v = 0
                omega = self.k_theta * self.warp_to_pi(goal_theta - odom_theta)
            else:
                v = self.k_rho * rho
                omega = self.k_alpha * alpha + self.k_beta * beta
            
            v, omega = self.scale_velocities(v, omega)
            
            if rho < self.translation_tolerance and abs(self.warp_to_pi(goal_theta - odom_theta)) < self.rotation_tolerance:
                # self.set_goal(None)
                for hook in self.goal_reached_hooks:
                    hook()
                # v = 0
                # omega = 0
            
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.pub_cmd_vel.publish(cmd)
        else:
            self.pub_cmd_vel.publish(Twist())

