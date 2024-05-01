import rospy


class Context:
    def __init__(self):
        pass

    def inject(self, target_object):
        for attr_name in dir(self):
            if not attr_name.startswith('__'): # 内建方法不可被注入
                setattr(target_object, attr_name, getattr(self, attr_name))


class ROSPublisherAndSubscriberPoolContext(Context):
    def __init__(self):
        super().__init__()
        self.publishers = {}
        self.subscribers = {}
    
    def add_publisher(self, topic, msg_type, **kwargs):
        if topic in self.publishers:
            raise ValueError('Publisher already exists for topic: %s' % topic)
        new_pub = rospy.Publisher(topic, msg_type, **kwargs)
        self.publishers[topic] = new_pub
        return new_pub
    
    def add_subscriber(self, topic, msg_type, **kwargs):
        if topic in self.subscribers:
            raise ValueError('Subscriber already exists for topic: %s' % topic)
        new_sub = rospy.Subscriber(topic, msg_type, **kwargs)
        self.subscribers[topic] = new_sub
        return new_sub
    
    def get_publisher(self, topic):
        return self.publishers[topic]

    def get_subscriber(self, topic):
        return self.subscribers[topic]
    
    def release_publisher(self, topic):
        if topic not in self.publishers:
            raise ValueError('Publisher not found for topic: %s' % topic)
        self.publishers[topic].unregister()
        del self.publishers[topic]
    
    def release_subscriber(self, topic):
        if topic not in self.subscribers:
            raise ValueError('Subscriber not found for topic: %s' % topic)
        self.subscribers[topic].unregister()
        del self.subscribers[topic]
    
    def quick_publish(self, topic, msg, **kwargs):
        self.publishers.get(topic, self.add_publisher(topic, type(msg), **kwargs)).publish(msg)

