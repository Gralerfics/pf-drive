import time

import rospy

import numpy as np


class Repeater:
    def __init__(self):
        self.init_parameters()
        self.init_topics()
    
    def init_parameters(self):
        # self.x = rospy.get_param('~x', x)
        pass
    
    def init_topics(self):
        pass
    
    def launch(self, node_name='repeater', anonymous=False):
        rospy.init_node(node_name, anonymous=anonymous)
        
        while not rospy.is_shutdown():
            rospy.loginfo('Repeater is running ...')
            time.sleep(1)
        # rospy.spin()

