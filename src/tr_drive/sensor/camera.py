import time
import threading

import rospy
from sensor_msgs.msg import Image

from tr_drive.util.debug import Debugger
from tr_drive.util.image import DigitalImage, ImageProcessor


"""
    用于接收摄像头图像信息, 对其进行处理 (grayscale & patch normalization), 调用注册的回调函数.
    
    register_x_hook():
        注册新的回调函数到列表.
    
    is_ready():
        为 True 时应保证可以: 获取 raw_image 和 processed_image; 执行回调函数.
        要求: raw_image 和 processed_image 皆不为 None.
    
    get_x():
        线程安全地获取成员.
"""
class Camera:
    def __init__(self,
        raw_image_topic: str,
        patch_size: list,
        resize: list,
        horizontal_fov: float,
        processed_image_topic: str = '/tr/camera/processed_image'
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'camera_debugger')
        self.image_received_hooks: list = []
        
        # public
        self.image_lock = threading.Lock()
        self.raw_image: DigitalImage = None
        self.processed_image: DigitalImage = None
        
        # parameters
        self.raw_image_topic = raw_image_topic
        self.patch_size = patch_size
        self.resize = resize
        self.horizontal_fov = horizontal_fov
        self.processed_image_topic = processed_image_topic
        
        # topics
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb, queue_size = 1)
    
    def __del__(self):
        self.sub_raw_image.unregister()
    
    def raw_image_cb(self, msg): # 订阅话题, 转为 DigitalImage 以及处理后存储; 若 ready 则执行回调函数.
        with self.image_lock:
            self.raw_image = DigitalImage(msg)
            self.processed_image = ImageProcessor.kernel_normalize(DigitalImage(msg).interpolate(*self.resize).grayscale(), self.patch_size)
            # self.debugger.publish(self.processed_image_topic, self.processed_image.to_Image(encoding = 'mono8'))
        
        if self.is_ready(): # 保证自身 ready 后再执行回调函数.
            for hook in self.image_received_hooks:
                hook()
                # hook(processed_image = self.processed_image)
    
    def register_image_received_hook(self, hook):
        self.image_received_hooks.append(hook)
    
    def is_ready(self):
        with self.image_lock:
            return self.raw_image is not None and self.processed_image is not None
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for image ...')
            time.sleep(0.2)
        rospy.loginfo('Camera is ready.')
    
    def set_params(self, **kwargs):
        with self.image_lock:
            self.patch_size = kwargs['patch_size'] if 'patch_size' in kwargs.keys() else self.patch_size
            self.resize = kwargs['resize'] if 'resize' in kwargs.keys() else self.resize
            self.horizontal_fov = kwargs['horizontal_fov'] if 'horizontal_fov' in kwargs.keys() else self.horizontal_fov
    
    def get_raw_image(self):
        with self.image_lock:
            return self.raw_image
    
    def get_processed_image(self):
        with self.image_lock:
            return self.processed_image

