import time

import rospy
from geometry_msgs.msg import Twist

from tr_drive.util.debug import Debugger


class GoalController:
    def __init__(self, namespace):
        self.init_parameters(namespace)
        self.init_topics()
        
        self.debugger: Debugger = Debugger(name = 'goal_controller_debugger')
    
    def init_parameters(self, namespace):
        self.cmd_vel_topic = rospy.get_param(namespace + '/cmd_vel_topic')
    
    def init_topics(self):
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 1)
    
    def is_ready(self):
        return True # TODO
    
    def wait_until_ready(self):
        while not self.is_ready():
            rospy.loginfo('Waiting for goal controller ...')
            time.sleep(0.1)
        rospy.loginfo('Goal controller is ready.')

