import os
import time

import numpy as np

import rospy
import tf

from . import t3d_ext


"""
    Generate time string
"""
def stamp_str():
    return time.strftime('%Y-%m-%d_%H:%M:%S')


"""
    a.get('b', {}).get('c', {}) ... .get('d', default)
"""
def fetch(d: dict, keys: list, default = None):
    obj = d
    for key in keys:
        if key in obj:
            obj = obj[key]
        else:
            return default
    return obj

"""
    List files in folder, sorted by filename number
"""
def get_filename_number(filename):
    try:
        res = int(filename.split('.')[0])
    except ValueError:
        res = -1
    return res

def get_numbered_file_list(folder: str):
    return [x for x in sorted(os.listdir(folder), key = lambda x: get_filename_number(x)) if get_filename_number(x) >= 0]


"""
    list 实现可访问元素的队列, 队首为 0
    效率相对不高, 且受元素大小影响, 仅用于较短且较不频繁的操作
"""
class ListQueue:
    def __init__(self, size = 5):
        self.size = size
        self.q = []
    
    def __getitem__(self, s):
        return self.q[s]
    
    def push(self, x):
        self.q.append(x)
        if len(self.q) > self.size:
            self.q.pop(0)
    
    def pop(self):
        if len(self.q) > 0:
            return self.q.pop(0)
        return None

    def is_full(self):
        return len(self.q) == self.size


"""
    ROS Context
    注意: 不要在 __init__ 中调用 rospy.init_node, 否则可能导致多次初始化
"""
class ROSContext:
    def __init__(self, name = 'context'):
        self.name = name

        self.publishers = {}
        self.subscribers = {}

        self.tf_broadcaster = None
        self.tf_listener = None # tf.TransformListener()

        self.service_proxies = {}
    
    def init_node(self, anonymous = True):
        rospy.init_node(self.name, anonymous = anonymous)
    
    def publish_topic(self, topic, msg, queue_size = 100): # queue_size 适当调大, 否则高速发布下可能丢包
        if topic not in self.publishers:
            self.publishers[topic] = rospy.Publisher(topic, type(msg), queue_size = queue_size)
            time.sleep(0.5) # 创建 Publisher 后需要等待一段时间, 否则可能会丢包
        self.publishers[topic].publish(msg)
    
    def subscribe_topic(self, topic, msg_type, callback, queue_size = None): # .spin() required
        if topic not in self.subscribers:
            self.subscribers[topic] = rospy.Subscriber(topic, msg_type, callback, queue_size = queue_size)

    def publish_tf(self, T_ab, frame_id_a, frame_id_b):
        if self.tf_broadcaster is None:
            self.tf_broadcaster = tf.TransformBroadcaster()
        t_ab, q_ab = t3d_ext.edt(T_ab), t3d_ext.edq_xyzw(T_ab)
        self.tf_broadcaster.sendTransform(
            t_ab,
            q_ab,
            rospy.Time.now(),
            frame_id_b, # child
            frame_id_a  # parent
        )
    
    def register_service(self, service, srv_type):
        if service not in self.service_proxies:
            self.service_proxies[service] = rospy.ServiceProxy(service, srv_type)
        rospy.wait_for_service(service)
    
    def call_service(self, service, request, wait_for_service = False, timeout = None):
        try:
            if wait_for_service:
                rospy.wait_for_service(service, timeout = timeout)
            return self.service_proxies[service](request)
        except rospy.ServiceException as e:
            rospy.logerr('Call service failed: %s' % e)
            return None
    
    def loginfo(self, msg):
        rospy.loginfo(msg)
    
    def logerr(self, msg):
        rospy.logerr(msg)
    
    def time(self):
        return rospy.Time.now()
    
    def is_shutdown(self):
        return rospy.is_shutdown()
    
    def spin(self):
        rospy.spin()
    
    def spin_once(self, duration = 0.01): # is that right?
        if not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(duration)

