import time
import math
import threading

import rospy
from geometry_msgs.msg import Twist

from tr_drive.util.debug import Debugger
from tr_drive.util.conversion import Frame

from tr_drive.sensor.odometry import Odom


"""
    goal 导向的控制器, 可暂停和恢复, 调用注册的回调函数 (e.g. arrived?).
    
    set_odometry():
        注册 odometry.
    
    is_ready():
        为 True 时方允许: 获取和设置 goal; ...
        要求: odometry 已注册且已 ready.
    
    (de)activate():
        暂停和恢复, odom_received 执行的前提.
    
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
    
    def is_ready(self):
        return self.odometry is not None and self.odometry.is_ready()
    
    def wait_until_ready(self):
        while not self.is_ready():
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
    
    def warp_to_pi(self, x):
        return ((x + math.pi) % (2 * math.pi)) - math.pi # TODO
    
    def rho_alpha_beta(self, dx, dy, theta, goal_theta):
        rho = math.sqrt(dx * dx + dy * dy)
        alpha = self.warp_to_pi(math.atan2(dy, dx) - theta)
        beta = self.warp_to_pi(-theta - alpha + goal_theta)
        return rho, alpha, beta

    def set_goal(self, goal: Frame):
        if not self.is_ready():
            return False
        
        with self.goal_lock:
            self.goal = goal
    
    def get_goal(self):
        if not self.is_ready():
            return False
        
        with self.goal_lock:
            return self.goal
    
    def odom_received(self, **args):
        if not self.is_ready() or not self.activated or self.goal is None:
            return False
        
        pass # TODO
        print('controller odom.')
        return True
    
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

