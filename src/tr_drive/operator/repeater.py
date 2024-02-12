import time

import rospy

import numpy as np

from tr_drive.util.debug import Debugger
from tr_drive.util.conversion import Frame
from tr_drive.util.image import DigitalImage, ImageProcessor

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera
from tr_drive.controller.goal_controller import GoalController


class Repeater:
    def __init__(self, namespace):
        self.ready = False
        self.debugger: Debugger = Debugger(name = 'repeater_debugger')
        
        self.first_image = None
        
        self.init_parameters(namespace)
        self.init_devices(namespace)
        self.ready = True
    
    def init_parameters(self, namespace):
        self.recording_path = rospy.get_param(namespace + '/persistent/recording_path')
    
    def init_devices(self, namespace):
        self.camera = Camera(namespace = namespace + '/camera')
        self.camera.register_image_received_hook(self.image_received)
        self.camera.wait_until_ready()
        self.odometry = Odom(namespace = namespace + '/odometry')
        self.odometry.register_odom_received_hook(self.odom_received)
        self.odometry.wait_until_ready()
        self.controller = GoalController(namespace = namespace + '/controller')
        self.controller.wait_until_ready()
    
    def image_received(self, **args):
        if self.ready:
            if self.first_image is None:
                self.first_image = args['image']
            current_image: DigitalImage = args['image']
            r, c = current_image.height, current_image.width
            
            offsets = np.arange(1 - c, c).astype(int)
            correlations = ImageProcessor.horizontal_NCC(current_image, self.first_image, offsets)
            values = np.clip((r - (np.array(correlations)) * r).astype(int), 0, r - 1)
            
            diagram_image = DigitalImage(np.array(np.zeros((r, 2 * c - 1, 1)), dtype = np.uint8))
            diagram_image.data[values, offsets + c - 1] = [255]
            diagram_image.data[:, np.argmax(correlations)] = [255]
            
            self.debugger.publish('diagram', diagram_image.to_Image(encoding = 'mono8'))

    def odom_received(self, **args):
        if self.ready:
            pass


# TODO: idea: 金字塔匹配辅助确认距离; 互相关加权，倾向小角度;