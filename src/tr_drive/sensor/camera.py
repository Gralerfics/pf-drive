import time
import threading

import rospy
from sensor_msgs.msg import Image

from tr_drive.util.debug import Debugger
from tr_drive.util.image import DigitalImage, ImageProcessor


# 接收摄像头图像信息，对其进行处理（grayscale & patch normalization），调用注册的回调函数；
# ready 表示回调已注册且已开始收到消息，ready 时方调用回调函数；
# 使用 get_ 方法获取成员，避免线程冲突。
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
        self.image_received_hook = None
        self.last_image_msg: Image = None
        
        # public
        self.raw_image: DigitalImage = None
        self.raw_image_lock = threading.Lock()
        self.processed_image: DigitalImage = None
        self.processed_image_lock = threading.Lock()
        
        # parameters
        self.raw_image_topic = raw_image_topic
        self.patch_size = patch_size
        self.resize = resize
        self.horizontal_fov = horizontal_fov
        self.processed_image_topic = processed_image_topic
        
        # topics
        self.init_topics()
    
    def init_topics(self):
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb, queue_size = 1)
    
    def raw_image_cb(self, msg):
        self.last_image_msg = msg
        
        # raw_image
        self.raw_image_lock.acquire()
        try:
            self.raw_image = DigitalImage(msg)
        finally:
            self.raw_image_lock.release()
        
        # processed_image
        self.processed_image_lock.acquire()
        try:
            self.processed_image = ImageProcessor.kernel_normalize(DigitalImage(msg).interpolate(*self.resize).grayscale(), self.patch_size)
            # self.debugger.publish(self.processed_image_topic, self.processed_image.to_Image(encoding = 'mono8'))
        finally:
            self.processed_image_lock.release()
        
        # hook
        if self.is_ready():
            self.image_received_hook(image = self.processed_image)
    
    def register_image_received_hook(self, hook):
        self.image_received_hook = hook
    
    def is_ready(self):
        return (self.last_image_msg is not None) and (self.image_received_hook is not None)
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for image ...')
            time.sleep(0.2)
        rospy.loginfo('Camera is ready.')
    
    def modify_raw_image_topic(self, topic): # 修改订阅的话题（参数服务器参数仅作为初始默认值，修改参数不通过参数服务器）
        if self.is_ready():
            self.last_image_msg = None
            self.raw_image_topic = topic
            self.sub_raw_image.unregister()
            self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb, queue_size = 1)
            return True
        else:
            return False
    
    def get_raw_image(self):
        if self.is_ready():
            self.raw_image_lock.acquire()
            res = self.raw_image
            self.raw_image_lock.release()
            return res
        else:
            return False
    
    def get_processed_image(self):
        if self.is_ready():
            self.processed_image_lock.acquire()
            res = self.processed_image
            self.processed_image_lock.release()
            return res
        else:
            return False

