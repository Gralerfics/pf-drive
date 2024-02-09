import rospy


class Debugger:
    def __init__(self, name = 'debugger'):
        self.name = name
        self.publishers = {}
    
    def publish(self, topic, msg):
        if topic not in self.publishers:
            self.publishers[topic] = rospy.Publisher(topic, type(msg), queue_size = 10) # TODO: queue_size.
        self.publishers[topic].publish(msg)

