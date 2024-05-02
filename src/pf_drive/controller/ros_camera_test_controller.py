import time

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from multinodes import Node

from pf_drive.util import ROSContext


class ROSCameraTestController(Node):
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)
        
        while not self.is_shutdown() and not ros.is_shutdown():
            if 'camera_image' not in self.io:
                time.sleep(0.1)
                continue

            if self.io['camera_image'].poll():
                cv_image = self.io['camera_image'].read()
                ros_image_msg = CvBridge().cv2_to_imgmsg(cv_image, encoding = "passthrough")

                ros.publish_topic('/test', ros_image_msg, queue_size = 1)

