import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from multinodes import Node


class ROSCamera(Node):
    def __init__(self, name, image_topic):
        super().__init__(name)
        self.image_topic = image_topic
    
    def run(self):
        rospy.init_node(self.name, anonymous = False)
        bridge = CvBridge()

        def image_callback(data):
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
            
            self.io['image'].write(cv_image)
        
        rospy.Subscriber(self.image_topic, Image, image_callback)
        rospy.spin()

