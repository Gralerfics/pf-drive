import time
import threading

import rospy
import tf

from webots_ros.srv import supervisor_get_from_defRequest, supervisor_get_from_defResponse, supervisor_get_from_def
from webots_ros.srv import node_get_positionRequest, node_get_positionResponse, node_get_position, node_get_orientationRequest, node_get_orientationResponse, node_get_orientation

from pf_drive.util.debug import Debugger
from pf_drive.util.geometry import Vec3, Quat, Frame

from pf_drive.sensor.odometry import Odom


"""
    用于获取全局定位 (作为真值参考等).
    几种类型:
        topic 类型需提供 topic, topic_type, odometry, fixed_frame;
        tf 类型需提供 odometry, fixed_frame;
        webots 类型需提供 robot_def, supervisor_srv, odometry, fixed_frame.
    传入时应保证 odometry 已 ready.
    
    register_x_hook():
        注册新的回调函数到列表.
    
    is_ready():
        为 True 时方允许: 获取 global_frame; 执行回调函数.
        要求: global_frame 不为 None.
    
    get_x():
        线程安全地获取成员.
"""
class GlobalLocator:
    def __init__(self,
        locator_type: str,
        odometry: Odom,
        fixed_frame_id: str,
        aligned_global_frame_id = 'aligned_map',
        **kwargs
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'global_locator_debugger')
        self.global_frame_received_hooks: list = []
        
        # public
        self.global_frame_lock = threading.Lock()
        self.global_frame: Frame = None # frame_id = map
        self.alignment_lock = threading.Lock()
        self.alignment: Frame = Frame()
        
        # parameters
        self.LOCATOR_TYPE_TOPIC = 'topic'
        self.LOCATOR_TYPE_TF = 'tf'
        self.LOCATOR_TYPE_WEBOTS = 'webots'

        self.locator_type = locator_type
        self.odometry: Odom = odometry
        self.fixed_frame_id = fixed_frame_id
        self.aligned_global_frame_id = aligned_global_frame_id
        self.params = kwargs

        self.odom_frame_id = self.odometry.get_odom_frame_id()
        
        # clients
        self.tf_broadcaster = tf.TransformBroadcaster() # 用于发布 map -> odom, 以及 map -> aligned_map 的变换.
        self.tf_listener = tf.TransformListener()
        
        if self.locator_type == self.LOCATOR_TYPE_TOPIC:
            self.topic = self.params['topic']
            self.topic_type = self.params['topic_type']

            # 用于订阅指定话题
            self.sub_topic = rospy.Subscriber(self.topic, self.topic_type, self.topic_type_cb, queue_size = 1)
        elif self.locator_type == self.LOCATOR_TYPE_TF:
            # 以 odom 的接收进行回调
            self.odometry.register_odom_received_hook(self.tf_type_cb)
        elif self.locator_type == self.LOCATOR_TYPE_WEBOTS:
            self.robot_def = self.params['robot_def']
            self.supervisor_srv = self.params['supervisor_srv']
            
            # 定义服务
            SERVICE_GET_FROM_DEF = self.supervisor_srv + '/get_from_def'
            SERVICE_GET_POSITION = self.supervisor_srv + '/node/get_position'
            SERVICE_GET_ORIENTATION = self.supervisor_srv + '/node/get_orientation'
            rospy.wait_for_service(SERVICE_GET_FROM_DEF)
            rospy.wait_for_service(SERVICE_GET_POSITION)
            rospy.wait_for_service(SERVICE_GET_ORIENTATION)
            self.srv_get_from_def = rospy.ServiceProxy(SERVICE_GET_FROM_DEF, supervisor_get_from_def)
            self.srv_get_position = rospy.ServiceProxy(SERVICE_GET_POSITION, node_get_position)
            self.srv_get_orientation = rospy.ServiceProxy(SERVICE_GET_ORIENTATION, node_get_orientation)

            # 获取 robot 节点句柄 (node)
            try:
                request = supervisor_get_from_defRequest(name = self.robot_def, proto = 0)
                response = self.srv_get_from_def(request)
                self.node_handler = response.node # 获取 node 句柄
            except rospy.ServiceException as e: # TODO
                rospy.logerr('Service call get_from_def failed.')

            # 以 odom 的接收进行回调
            self.odometry.register_odom_received_hook(self.webots_type_cb)
        else:
            raise ValueError('Invalid locator type.')
    
    def __del__(self):
        if self.locator_type == self.LOCATOR_TYPE_TOPIC:
            self.sub_topic.unregister()
        elif self.locator_type == self.LOCATOR_TYPE_TF:
            pass
        elif self.locator_type == self.LOCATOR_TYPE_WEBOTS:
            pass
        else:
            raise ValueError('Invalid locator type.')
    
    def publish_map_frame(self, odom_frame: Frame, global_frame: Frame, odom_frame_id = 'odom', global_frame_id = 'map'):
        # 给出某点在 map 和 odom 下的坐标, 计算并发布 map -> odom 变换.
        # 由里程计信息和全局信息得到 /map 和 /odom 关系并发布; 此处 odom_frame 由 get_odom() 所获, frame_id 为 odom.
        T_map_odom = global_frame * odom_frame.I # T_map_odom = T_map_p * T_p_odom = T_map_p * Inv(T_odom_p)
        self.tf_broadcaster.sendTransform(
            T_map_odom.t.to_list(),
            T_map_odom.q.to_list(),
            rospy.Time.now(),
            odom_frame_id,
            global_frame_id
        )
    
    def publish_aligned_map_frame(self, global_frame_id = 'map', aligned_global_frame_id = 'aligned_map'):
        alignment = self.get_alignment()
        self.tf_broadcaster.sendTransform(
            alignment.t.to_list(),
            alignment.q.to_list(),
            rospy.Time.now(),
            aligned_global_frame_id,
            global_frame_id
        )
    
    def get_map_to_odom_frame(self):
        # 统一接收 map -> odom 变换 (T_map_odom), 可能来自 tf 模式的 tf 树或者另外两个模式发布出去的变换.
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.fixed_frame_id, self.odom_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        return Frame(Vec3(trans), Quat(rot))

    def call_hooks(self):
        if self.is_ready(): # 保证自身 ready 后再执行回调函数.
            for hook in self.global_frame_received_hooks:
                hook()
    
    """
        各模式 callback 的要求:
            计算 self.global_frame;
            若无则发布 map -> odom 变换;
            发布 map -> aligned_map 变换;
            执行回调函数.
    """

    def topic_type_cb(self, msg): # topic 模式
        # 订阅话题直接获取全局坐标
        with self.global_frame_lock:
            self.global_frame = Frame(msg)
        
        # 计算并发布 map -> odom 变换
        self.publish_map_frame(self.odometry.get_odom(), self.global_frame, self.odom_frame_id, self.fixed_frame_id)

        # 发布 map -> aligned_map 变换
        self.publish_aligned_map_frame(self.fixed_frame_id, self.aligned_global_frame_id)
        
        # 调用回调函数
        self.call_hooks()
    
    def tf_type_cb(self): # tf 模式
        # 获取 map -> odom 的变换, 计算全局坐标
        T_odom_p = self.odometry.get_odom()
        T_map_odom = self.get_map_to_odom_frame()
        with self.global_frame_lock:
            self.global_frame = T_map_odom * T_odom_p
        
        # 发布 map -> aligned_map 变换
        self.publish_aligned_map_frame(self.fixed_frame_id, self.aligned_global_frame_id)
        
        # 调用回调函数
        self.call_hooks()
    
    def webots_type_cb(self): # webots 模式
        # 通过服务获取全局坐标
        try:
            position_request = node_get_positionRequest(node = self.node_handler)
            position_response = self.srv_get_position(position_request)
            t = Vec3(position_response.position.x, position_response.position.y, position_response.position.z)
            orientation_request = node_get_orientationRequest(node = self.node_handler)
            orientation_response = self.srv_get_orientation(orientation_request)
            q = Quat(orientation_response.orientation.x, orientation_response.orientation.y, orientation_response.orientation.z, orientation_response.orientation.w)
            with self.global_frame_lock:
                self.global_frame = Frame(t, q)
        except rospy.ServiceException as e: # TODO
            rospy.logerr('Service call node_get_position or node_get_orientation failed.')
        
        # 计算并发布 map -> odom 变换
        self.publish_map_frame(self.odometry.get_odom(), self.global_frame, self.odom_frame_id, self.fixed_frame_id)

        # 发布 map -> aligned_map 变换
        self.publish_aligned_map_frame(self.fixed_frame_id, self.aligned_global_frame_id)

        # 调用回调函数
        self.call_hooks()
    
    def register_global_frame_received_hook(self, hook):
        self.global_frame_received_hooks.append(hook)
    
    def is_ready(self):
        with self.global_frame_lock:
            return self.global_frame is not None
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for global locator ...')
            time.sleep(0.2)
        rospy.loginfo('Global locator is ready.')
    
    def align_odom_with_global(self, odom_frame: Frame, global_frame: Frame): # TODO: global_locator 中应该专注 global, 设计上不便涉及 odom? 或者这里设计为一个工具函数, 体现在名称或类型中?
        # 目的: 计算 alignment, 令 map 下的 global_frame 在左乘 alignment 后与 odom 下的 odom_frame 重合.
            # T_am_p 也即这里的 global_frame 应当是 frame_id = aligned_map 的; T_odom_p 也即这里的 odom_frame 应当是 frame_id = odom 的.
            # 有等式 T_map_am * T_am(i.e. aligned_map)_p = T_map_odom * T_odom_p;
            # 故 alignment = T_map_am = T_map_odom * T_odom_p * Inv(T_am_p) = T_map_odom * odom_frame * Inv(global_frame).
        T_map_odom = self.get_map_to_odom_frame()
        with self.alignment_lock:
            self.alignment = T_map_odom * odom_frame * global_frame.I
    
    def align_biased_odom_with_global(self, biased_odom_frame: Frame, global_frame: Frame):
        T_map_odom = self.get_map_to_odom_frame()
        T_odom_biased_odom = self.odometry.get_bias_inv().I
        with self.alignment_lock:
            self.alignment = T_map_odom * T_odom_biased_odom * biased_odom_frame * global_frame.I
    
    def get_alignment(self):
        with self.alignment_lock:
            return self.alignment
    
    def get_global_frame_id(self):
        return self.fixed_frame_id
    
    def get_global_frame(self):
        with self.global_frame_lock:
            return self.global_frame
    
    def get_aligned_global_frame_id(self):
        return self.aligned_global_frame_id
    
    def get_aligned_global_frame(self):
        with self.global_frame_lock:
            return self.get_alignment() * self.global_frame

