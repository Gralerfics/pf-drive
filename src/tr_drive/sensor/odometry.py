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
        为 True 时方允许: 获取 biased_odom; 执行回调函数.
        要求: 已开始收到消息.
    
    modify_x_topic():
        动态修改 topic.
    
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
        self.last_odom_msg: Odometry = None
        self.bias: Odometry = Odometry()
        
        # public
        self.biased_odom = None
        self.biased_odom_lock = threading.Lock()
        
        # parameters
        self.odom_topic = odom_topic
        self.processed_odom_topic = processed_odom_topic
        
        # topics
        self.init_topics()
    
    def init_topics(self):
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size = 1) # TODO: queue_size
    
    def odom_cb(self, msg: Odometry):
        self.last_odom_msg = msg
        
        # biased_odom
        with self.biased_odom_lock:
            self.biased_odom = Frame(self.bias).I * Frame(msg)
        
        # hook
        if self.is_ready():
            for hook in self.odom_received_hooks:
                hook(odom = self.biased_odom)
    
    def register_odom_received_hook(self, hook):
        self.odom_received_hooks.append(hook)
        
    def is_ready(self):
        return self.last_odom_msg is not None
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for odometry ...')
            time.sleep(0.2)
        rospy.loginfo('Odometry is ready.')
    
    def modify_odom_topic(self, topic):
        self.odom_topic = topic
        self.sub_odom.unregister()
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        return True
    
    def get_biased_odom(self):
        if not self.is_ready():
            return False
        
        with self.biased_odom_lock:
            res = self.biased_odom
        return res
    
    def reset(self): # 重置零偏
        self.bias = Odometry()
        return True
    
    def zeroize(self): # 当前点设为零点
        self.bias = self.last_odom_msg
        return True

