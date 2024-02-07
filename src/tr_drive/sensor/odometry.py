import time

import rospy
from nav_msgs.msg import Odometry

from tr_drive.util.conversion import Frame


class Odometry:
    def __init__(self, node_name='tr_odometry', anonymous=False):
        rospy.init_node(node_name, anonymous=anonymous)
        self.init_parameters()
        self.init_topics()
        
        self.last_odom: Odometry = None
        self.bias: Odometry = Odometry()
    
    def init_parameters(self):
        self.odom_topic = rospy.get_param('/tr/odometry/odom_topic')
        self.processed_odom_topic = rospy.get_param('/tr/odometry/processed_odom_topic')
    
    def init_topics(self):
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.pub_processed_odom = rospy.Publisher(self.processed_odom_topic, Odometry, queue_size=1)
    
    def spin(self):
        rospy.spin()
    
    def odom_cb(self, msg: Odometry):
        self.last_odom = msg
        self.pub_processed_odom.publish(msg - self.bias) # TODO, minus operator is not defined for Odometry
    
    def reset(self):
        self.last_odom = None
        self.bias = Odometry()
    
    def zeroize(self):
        if self.last_odom is not None:
            self.bias = self.last_odom
            return True
        else:
            return False

