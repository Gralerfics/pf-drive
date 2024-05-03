import time

import numpy as np

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from multinodes import Node

from pf_drive.util import ROSContext


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
    
    def patch_normalize(self, img, patch_size):
        patch_radius = (patch_size - 1) // 2
        r, c = img.shape
        
        img_pad = np.pad(np.float64(img), patch_radius, 'constant', constant_values = np.nan)
        
        kernels = np.lib.stride_tricks.as_strided(img_pad, (r, c, patch_size, patch_size), img_pad.strides + img_pad.strides).reshape(r, c, -1)
        kernel_mean = np.nanmean(kernels, axis = 2).reshape(r, c)
        kernel_std = np.nanstd(kernels, axis = 2).reshape(r, c)
        with np.errstate(divide = 'ignore', invalid = 'ignore'):
            res = (img - kernel_mean) / kernel_std
        res[np.isnan(res)] = 0.0
        np.clip(res, -1.0, 1.0, out = res)
        
        return np.uint8((res + 1.0) / 2 * 255)
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)
        bridge = CvBridge()

        def image_callback(data):
            if 'image' not in self.io:
                return

            img = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
            img = cv2.resize(img, self.resize, interpolation = cv2.INTER_AREA)
            img = self.patch_normalize(img, self.patch_size)
            
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
    
    def patch_normalize(self, img, patch_size):
        patch_radius = (patch_size - 1) // 2
        r, c = img.shape
        
        img_pad = np.pad(np.float64(img), patch_radius, 'constant', constant_values = np.nan)
        
        kernels = np.lib.stride_tricks.as_strided(img_pad, (r, c, patch_size, patch_size), img_pad.strides + img_pad.strides).reshape(r, c, -1)
        kernel_mean = np.nanmean(kernels, axis = 2).reshape(r, c)
        kernel_std = np.nanstd(kernels, axis = 2).reshape(r, c)
        with np.errstate(divide = 'ignore', invalid = 'ignore'):
            res = (img - kernel_mean) / kernel_std
        res[np.isnan(res)] = 0.0
        np.clip(res, -1.0, 1.0, out = res)
        
        return np.uint8((res + 1.0) / 2 * 255)
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)
        bridge = CvBridge()

        def image_callback(data):
            if 'command' not in self.io:
                return

            self.raw_img = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
            self.proc_img = cv2.cvtColor(self.raw_img, cv2.COLOR_RGBA2GRAY)
            self.proc_img = cv2.resize(self.proc_img, self.resize, interpolation = cv2.INTER_AREA)
            self.proc_img = self.patch_normalize(self.proc_img, self.patch_size)
            
            self.handle_rpc_once(self.io['command'], block = False) # 此处同步处理，保证已有数据且完整
        
        ros.subscribe_topic(self.image_topic, Image, image_callback)
        ros.spin()

