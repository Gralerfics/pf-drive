import time

import rospy


class Teacher:
    def __init__(self, node_name='tr_teacher', anonymous=False):
        rospy.init_node(node_name, anonymous=anonymous)
        self.init_parameters()
        self.init_topics()
    
    def init_parameters(self):
        # self.x = rospy.get_param('~x', x)
        pass
    
    def init_topics(self):
        pass
    
    def spin(self):
        rospy.spin()

