import time

import cv_bridge

from multinodes import Node

from pf_drive.util import ROSContext


"""
    `camera_image`, input (shared_object)
        format: cv2 image (np.array)
    `odom`, input (pipe)
        format: 4x4 np.array
    `actuator_command`, output (pipe)
        format:
            (type = 'vw', v, w)
            (type = 'vphi', v, phi)
"""
class RepeatController(Node):
    def __init__(self, name):
        super().__init__(name)

    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)

        bridge = cv_bridge.CvBridge()
        
        while not ros.is_shutdown():
            """
                Test code
            """
            if 'camera_image' in self.io:
                self.io['actuator_command'].write(('vphi', 0.3, 0.02))
            
            if 'odom' in self.io and self.io['odom'].poll():
                print(self.io['odom'].read())
            
            if 'camera_image' in self.io and self.io['camera_image'].poll():
                image_msg = bridge.cv2_to_imgmsg(self.io['camera_image'].read(), 'passthrough')
                ros.publish_topic('/test_image', image_msg)

