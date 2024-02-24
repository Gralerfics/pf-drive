import time

import rospy


class Debugger:
    def __init__(self, name = 'debugger'):
        self.name = name
        self.publishers = {}
    
    def publish(self, topic, msg, queue_size = 100): # queue_size 适当调大, 否则高速发布下可能丢包.
        if topic not in self.publishers:
            self.publishers[topic] = rospy.Publisher(topic, type(msg), queue_size = queue_size)
            time.sleep(0.5) # 等待一小会时间, 否则前半段 rviz 似乎存在收不到的可能.
        self.publishers[topic].publish(msg)

