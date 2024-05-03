import time

import rospy
import tf

from pf_drive.util import t3d_ext


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
            # time.sleep(0.5) # 等待一小会时间, 否则前半段 rviz 似乎存在收不到的可能
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

