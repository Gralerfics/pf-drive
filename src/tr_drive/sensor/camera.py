import time

import rospy
from sensor_msgs.msg import Image


class Camera:
    def __init__(self, node_name='tr_camera', anonymous=False):
        rospy.init_node(node_name, anonymous=anonymous)
        self.init_parameters()
        self.init_topics()
    
    def init_parameters(self):
        self.raw_image_topic = rospy.get_param('/tr/camera/raw_image_topic')
        self.processed_image_topic = rospy.get_param('/tr/camera/processed_image_topic')
    
    def init_topics(self):
        self.sub_raw_image = rospy.Subscriber(self.raw_image_topic, Image, self.raw_image_cb)
        self.pub_processed_image = rospy.Publisher(self.processed_image_topic, Image, queue_size=1)
    
    def spin(self):
        rospy.spin()
    
    def raw_image_cb(self, msg):
        rospy.loginfo('Camera received raw image ...')
        self.pub_processed_image.publish(msg)

