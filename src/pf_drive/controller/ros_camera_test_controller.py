import time

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from multinodes import Node


class ROSCameraTestController(Node):
    def run(self):
        rospy.init_node(self.name, anonymous = False)
        publisher = rospy.Publisher("/test", Image, queue_size = 1)
        while not self.is_shutdown():
            if self.io['camera_image'].poll():
                cv_image = self.io['camera_image'].read()
                ros_image_msg = CvBridge().cv2_to_imgmsg(cv_image, encoding = "passthrough")
                publisher.publish(ros_image_msg)

