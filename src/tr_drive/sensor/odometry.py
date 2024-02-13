import time
import threading

import rospy
from nav_msgs.msg import Odometry

from tr_drive.util.debug import Debugger
from tr_drive.util.conversion import Frame


# 接收里程计信息，对其进行处理（零偏），调用注册的回调函数；
# ready 表示回调已注册且已开始收到消息，ready 时方调用回调函数；
# 后面的应用类方法，默认前提是已经 ready；
# 使用 get_ 方法获取成员，避免线程冲突。
class Odom:
    def __init__(self,
        odom_topic: str,
        processed_odom_topic: str = '/tr/odometry/processed'
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'odometry_debugger')
        self.odom_received_hook = None
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
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
    
    def odom_cb(self, msg: Odometry):
        self.last_odom_msg = msg
        
        # biased_odom
        with self.biased_odom_lock:
            self.biased_odom = Frame(self.bias).I * Frame(msg)
        
        # hook
        if self.is_ready():
            self.odom_received_hook(odom = self.biased_odom)
    
    def register_odom_received_hook(self, hook):
        self.odom_received_hook = hook
        
    def is_ready(self):
        return (self.last_odom_msg is not None) and (self.odom_received_hook is not None)
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for odometry ...')
            time.sleep(0.2)
        rospy.loginfo('Odometry is ready.')
    
    def modify_odom_topic(self, topic):
        if self.is_ready():
            self.last_odom_msg = None
            self.odom_topic = topic
            self.sub_odom.unregister()
            self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
            return True
        else:
            return False
    
    def get_biased_odom(self):
        if self.is_ready():
            with self.biased_odom_lock:
                res = self.biased_odom
            return res
        else:
            return False
    
    def reset(self): # 重置零偏
        if self.is_ready():
            self.bias = Odometry()
            return True
        else:
            return False
    
    def zeroize(self): # 当前点设为零点
        if self.is_ready():
            self.bias = self.last_odom_msg
            return True
        else:
            return False

