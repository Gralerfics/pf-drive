import time

import numpy as np

from sensor_msgs.msg import Image

import cv2

from multinodes import Node

from pf_drive.util import ROSContext
from pf_drive.util.img import Image_to_np, patch_normalize


"""
    `image`, output (shared_object)
        format: cv2 image (np.array)
"""
class ROSCameraWithProcessingAndSending(Node):
    def __init__(self, name, image_topic, resize, patch_size):
        super().__init__(name)
        self.image_topic = image_topic
        self.resize = resize
        self.patch_size = patch_size
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)

        def image_callback(data):
            if 'image' not in self.io:
                return

            img = Image_to_np(data)
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
            img = cv2.resize(img, self.resize, interpolation = cv2.INTER_AREA)
            img = patch_normalize(img, self.patch_size)
            
            self.io['image'].write(img)
        
        ros.subscribe_topic(self.image_topic, Image, image_callback)
        ros.spin()


"""
    `command`, input (rpc)
"""
class ROSCameraWithProcessingAndSaving(Node):
    def __init__(self, name, image_topic, resize, patch_size):
        super().__init__(name)
        self.image_topic = image_topic
        self.resize = resize
        self.patch_size = patch_size

        self.raw_img = None
        self.proc_img = None
    
    # @rpc
    def save_image(self, raw_img_path, proc_img_path):
        if self.raw_img is not None:
            cv2.imwrite(raw_img_path, self.raw_img)
        if self.proc_img is not None:
            cv2.imwrite(proc_img_path, self.proc_img)
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)

        def image_callback(data):
            if 'command' not in self.io:
                return

            self.raw_img = Image_to_np(data)
            self.proc_img = cv2.cvtColor(self.raw_img, cv2.COLOR_RGBA2GRAY)
            self.proc_img = cv2.resize(self.proc_img, self.resize, interpolation = cv2.INTER_AREA)
            self.proc_img = patch_normalize(self.proc_img, self.patch_size)
            
            self.handle_rpc_once(self.io['command'], block = False) # 此处同步处理，保证已有数据且完整
        
        ros.subscribe_topic(self.image_topic, Image, image_callback)
        ros.spin()

