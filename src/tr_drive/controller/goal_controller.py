import time

import rospy
from geometry_msgs.msg import Twist

from tr_drive.util.debug import Debugger


class GoalController:
    def __init__(self,
        cmd_vel_topic: str
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'goal_controller_debugger')
        
        # parameters
        self.cmd_vel_topic = cmd_vel_topic
        
        # topics
        self.init_topics()
    
    def init_topics(self):
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 1)
    
    def is_ready(self):
        return True # TODO
    
    def wait_until_ready(self):
        while not self.is_ready():
            rospy.loginfo('Waiting for goal controller ...')
            time.sleep(0.1)
        rospy.loginfo('Goal controller is ready.')

    def modify_cmd_vel_topic(self, topic):
        if self.is_ready():
            # TODO, ready -> False
            self.cmd_vel_topic = topic
            self.pub_cmd_vel.unregister()
            self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 1)
            return True
        else:
            return False

