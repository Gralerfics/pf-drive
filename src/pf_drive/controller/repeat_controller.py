import time

from multinodes import Node

from pf_drive.util import ROSContext


"""
    `processed_image`, input (shared_object)
        format: cv2 image (np.array)
    `odom`, input (pipe)
        format: 4x4 np.array
    `record`, input (queue)
        format: (image, odom)
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
        
        while not ros.is_shutdown():
            if 'processed_image' not in self.io or 'odom' not in self.io or 'actuator_command' not in self.io or 'record' not in self.io:
                time.sleep(0.1)
                continue

            if self.io['processed_image'].poll() and self.io['odom'].poll() and self.io['record'].poll():
                image = self.io['processed_image'].read()
                odom = self.io['odom'].read()

                time.sleep(1)
                self.io['record'].read()

