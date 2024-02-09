import time

import rospy

import numpy as np

from tr_drive.util.conversion import Frame
from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera
from tr_drive.controller.goal_controller import GoalController


class Repeater:
    def __init__(self, namespace):
        self.init_parameters()
        self.init_devices(namespace)
    
    def init_parameters(self):
        pass
    
    def init_devices(self, namespace):
        self.odometry = Odom(namespace = namespace + '/odometry')
        self.camera = Camera(namespace = namespace + '/camera')
        self.controller = GoalController(namespace = namespace + '/controller')
        self.odometry.register_odom_received_hook(self.odom_received)
        self.camera.register_image_received_hook(self.image_received)
        self.odometry.wait_until_ready()
        self.odometry.zeroize()
        self.camera.wait_until_ready()
        self.controller.wait_until_ready()
    
    def image_received(self, **args):
        pass

    def odom_received(self, **args):
        pass


# TODO: idea, 金字塔匹配辅助确认距离