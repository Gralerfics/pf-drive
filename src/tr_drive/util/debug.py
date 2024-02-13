import time

import rospy


class Debugger:
    def __init__(self, name = 'debugger'):
        self.name = name
        self.publishers = {}
    
    def publish(self, topic, msg, queue_size = 100):
        if topic not in self.publishers:
            self.publishers[topic] = rospy.Publisher(topic, type(msg), queue_size = queue_size) # TODO: queue_size.
            time.sleep(0.5) # TODO
        self.publishers[topic].publish(msg)

