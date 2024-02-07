import time

import rospy
from sensor_msgs.msg import Image

from tr_drive.util.image import DigitalImage


# TODO (refactor): 不独立获取 ROS Parameter.
class Camera:
    def __init__(self):
        self.init_parameters()
        self.init_topics()
        
        self.last_image_msg: Image = None
        self.image_received_hook = None
    
    def init_parameters(self):
        self.raw_image_topic = rospy.get_param('/tr/camera/raw_image_topic')
        self.processed_image_topic = rospy.get_param('/tr/camera/processed_image_topic')
    
    def init_topics(self):
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb)
    
    def raw_image_cb(self, msg):
        self.last_image_msg = msg
        # processed_image = TODO: process and obtain DI
        if self.image_received_hook is not None:
            self.image_received_hook() # (image = processed_image)
    
    def register_image_received_hook(self, hook):
        self.image_received_hook = hook
    
    def is_ready(self):
        return (self.last_image_msg is not None) and (self.image_received_hook is not None)
    
    def wait_until_ready(self):
        while not self.is_ready():
            rospy.loginfo('Waiting for image ...')
            time.sleep(0.1)

