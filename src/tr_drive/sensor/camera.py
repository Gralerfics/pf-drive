import time

import rospy
from sensor_msgs.msg import Image

from tr_drive.util.debug import Debugger
from tr_drive.util.image import DigitalImage, ImageProcessor


class Camera:
    def __init__(self, namespace):
        self.init_parameters(namespace)
        self.init_topics()
        
        self.image_received_hook = None
        
        self.last_image_msg: Image = None
        self.processed_image: DigitalImage = None
        
        self.debugger: Debugger = Debugger(name = 'camera_debugger')
    
    def init_parameters(self, namespace):
        self.raw_image_topic = rospy.get_param(namespace + '/raw_image_topic')
        self.processed_image_topic = rospy.get_param(namespace + '/processed_image_topic')
        self.patch_size = rospy.get_param(namespace + '/patch_size')
        self.resize = rospy.get_param(namespace + '/resize')
        self.horizontal_fov = rospy.get_param(namespace + '/horizontal_fov')
    
    def init_topics(self):
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb, queue_size = 1)
    
    def raw_image_cb(self, msg):
        self.last_image_msg = msg
        
        self.processed_image = ImageProcessor.kernel_normalize(DigitalImage(msg).interpolate(*self.resize).grayscale(), self.patch_size)
        self.debugger.publish(self.processed_image_topic, self.processed_image.to_Image(encoding = 'mono8'))
        
        if self.image_received_hook is not None:
            self.image_received_hook(image = self.processed_image)
    
    def register_image_received_hook(self, hook):
        self.image_received_hook = hook
    
    def is_ready(self):
        return (self.last_image_msg is not None) and (self.image_received_hook is not None)
    
    def wait_until_ready(self):
        while not self.is_ready():
            rospy.loginfo('Waiting for image ...')
            time.sleep(0.1)
        rospy.loginfo('Camera is ready.')

