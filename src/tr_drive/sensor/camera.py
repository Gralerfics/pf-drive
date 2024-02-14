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
        为 True 时方允许: 获取 raw_image 和 processed_image; 重置; 置零; 执行回调函数.
        要求: 已开始收到消息.
    
    modify_x_topic():
        动态修改 topic.
    
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
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb, queue_size = 1) # TODO: queue_size
    
    def raw_image_cb(self, msg):
        self.last_image_msg = msg
        
        # raw_image
        with self.raw_image_lock:
            self.raw_image = DigitalImage(msg)
        
        # processed_image
        with self.processed_image_lock:
            self.processed_image = ImageProcessor.kernel_normalize(DigitalImage(msg).interpolate(*self.resize).grayscale(), self.patch_size)
            # self.debugger.publish(self.processed_image_topic, self.processed_image.to_Image(encoding = 'mono8'))
        
        # hook
        if self.is_ready():
            for hook in self.image_received_hooks:
                hook(image = self.processed_image)
    
    def register_image_received_hook(self, hook):
        self.image_received_hooks.append(hook)
    
    def is_ready(self):
        return self.last_image_msg is not None
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for image ...')
            time.sleep(0.2)
        rospy.loginfo('Camera is ready.')
    
    def modify_raw_image_topic(self, topic): # 修改订阅的话题（参数服务器参数仅作为初始默认值，修改参数不通过参数服务器）
        if not self.is_ready():
            return False
        
        self.last_image_msg = None
        self.raw_image_topic = topic
        self.sub_raw_image.unregister()
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb, queue_size = 1)
        return True
    
    def get_raw_image(self):
        if not self.is_ready():
            return False
        
        with self.raw_image_lock:
            res = self.raw_image
        return res
    
    def get_processed_image(self):
        if not self.is_ready():
            return False
        
        with self.processed_image_lock:
            res = self.processed_image
        return res

