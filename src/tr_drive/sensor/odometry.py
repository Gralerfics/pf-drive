import time

import rospy
from nav_msgs.msg import Odometry

from tr_drive.util.conversion import Frame


# TODO (refactor): 不独立获取 ROS Parameter.
class Odom:
    def __init__(self):
        self.init_parameters()
        self.init_topics()
        
        self.last_odom_msg: Odometry = None
        self.bias: Odometry = Odometry()
        
        self.biased_odom = None
        
        self.odom_received_hook = None
    
    def init_parameters(self):
        self.odom_topic = rospy.get_param('/tr/odometry/odom_topic')
        self.processed_odom_topic = rospy.get_param('/tr/odometry/processed_odom_topic')
    
    def init_topics(self):
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
    
    def odom_cb(self, msg: Odometry):
        self.last_odom_msg = msg
        self.biased_odom = Frame(self.bias).I * Frame(msg)
        if self.odom_received_hook is not None:
            self.odom_received_hook(odom = self.biased_odom)
    
    def register_odom_received_hook(self, hook):
        self.odom_received_hook = hook
    
    def reset(self):
        self.last_odom_msg = None
        self.bias = Odometry()
        
    def is_ready(self):
        return (self.last_odom_msg is not None) and (self.odom_received_hook is not None)
    
    def wait_until_ready(self):
        while not self.is_ready():
            rospy.loginfo('Waiting for odometry ...')
            time.sleep(0.1)
    
    def zeroize(self):
        if self.last_odom_msg is not None:
            self.bias = self.last_odom_msg
            return True
        else:
            return False

