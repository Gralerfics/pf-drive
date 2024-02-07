import time

import rospy

import numpy as np


class Repeater:
    def __init__(self, node_name='tr_repeater', anonymous=False):
        rospy.init_node(node_name, anonymous=anonymous)
        self.init_parameters()
        self.init_topics()
    
    def init_parameters(self):
        pass
    
    def init_topics(self):
        pass
    
    def spin(self):
        rospy.spin()

