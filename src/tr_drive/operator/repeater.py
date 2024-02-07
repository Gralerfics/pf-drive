import time

import rospy

import numpy as np

from tr_drive.util.conversion import Frame
from tr_drive.sensor.camera import Camera
from tr_drive.sensor.odometry import Odom


# TODO (refactor): 不独立获取 ROS Parameter, 由 repeater_node 传入.
class Repeater:
    def __init__(self):
        self.init_parameters()
        self.init_topics()
        self.init_devices()
    
    def init_parameters(self):
        pass
    
    def init_topics(self):
        pass
    
    def init_devices(self):
        self.camera = Camera()
        self.odometry = Odom()
        
        def image_received(**args):
            rospy.loginfo('Received image: {}'.format(self.camera.last_image_msg))

        def odom_received(**args):
            rospy.loginfo('Received odometry: {}'.format(self.odometry.biased_odom))
            # rospy.loginfo('Received odometry: {}'.format(args['odom']))
        
        self.camera.register_image_received_hook(image_received)
        self.odometry.register_odom_received_hook(odom_received)
        
        self.odometry.wait_until_ready()
        self.odometry.zeroize()
        self.camera.wait_until_ready()

