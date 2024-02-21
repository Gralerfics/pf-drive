import time
import threading

import rospy
from nav_msgs.msg import Odometry

from tr_drive.util.debug import Debugger
from tr_drive.util.conversion import Frame


"""
    用于接收里程计信息, 对其进行处理 (零偏), 调用注册的回调函数.
    
    register_x_hook():
        注册新的回调函数到列表.
    
    is_ready():
        为 True 时应保证可以: 获取 odom_msg, odom 和 biased_odom; 执行回调函数.
        要求: odom_msg, bias_inv 和 odom 皆不为 None.
        注:
            不可在 lock 内调用;
            ready 后不再会变为 False, 即话题等参数不再允许修改, 但可以销毁, 故应重载 __del__ 析构函数;
            is_ready 是供外界调用的, 仅在 callback 中存在部分内部调用.
    
    get_x():
        线程安全地获取成员.
"""
class Odom:
    def __init__(self,
        odom_topic: str,
        processed_odom_topic: str = '/tr/odometry/processed'
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'odometry_debugger')
        self.odom_received_hooks: list = []
        
        # public
        self.odom_lock = threading.Lock()
        self.odom_msg: Odometry = None
        self.bias_inv: Frame = Frame()
        self.odom: Frame = None
        
        # parameters
        self.odom_topic = odom_topic
        self.processed_odom_topic = processed_odom_topic
        
        # topics
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size = 1) # TODO: queue_size
    
    def __del__(self):
        self.sub_odom.unregister()
    
    def odom_cb(self, msg: Odometry): # 订阅话题, 转为 Frame 并存储; 若 ready 则执行回调函数.
        with self.odom_lock:
            self.odom_msg = msg
            self.odom = Frame(msg)
        
        if self.is_ready(): # 保证自身 ready 后再执行回调函数.
            for hook in self.odom_received_hooks:
                hook()
                # hook(biased_odom = self.get_biased_odom())
    
    def register_odom_received_hook(self, hook):
        self.odom_received_hooks.append(hook)
    
    def is_ready(self):
        with self.odom_lock:
            return self.odom_msg is not None and self.odom is not None and self.bias_inv is not None
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for odometry ...')
            time.sleep(0.2)
        rospy.loginfo('Odometry is ready.')
    
    def reset(self): # 重置零偏
        with self.odom_lock:
            self.bias_inv = Frame()
    
    def zeroize(self): # 将当前点设为零
        with self.odom_lock:
            self.bias_inv = self.odom.I
    
    def get_odom_msg(self):
        with self.odom_lock:
            return self.odom_msg
    
    def get_odom(self):
        with self.odom_lock:
            return self.odom
    
    def get_biased_odom(self):
        with self.odom_lock:
            return self.bias_inv * self.odom

