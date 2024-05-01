import time

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from multinodes import Node


class ROSCamera(Node):
    def __init__(self, name, is_shutdown_event, image_topic):
        super().__init__(name, is_shutdown_event)
        self.image_topic = image_topic
    
    def run(self):
        rospy.init_node(self.name, anonymous = False)
        bridge = CvBridge()

        def image_callback(data):
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
            cv_image = cv2.resize(cv_image, (150, 50), interpolation = cv2.INTER_AREA)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2GRAY)
            self.io['image'].write(cv_image)
        
        rospy.Subscriber(self.image_topic, Image, image_callback)
        rospy.spin()

