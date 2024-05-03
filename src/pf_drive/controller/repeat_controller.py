import time

import numpy as np

from multinodes import Node

from pf_drive.util import t3d_ext, ROSContext, ListQueue


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
class BaselineRepeatController(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name)
        
        # 参数
        self.match_offset_radius = kwargs['match_offset_radius'] # 暂时没用到
        self.along_path_radius = kwargs['along_path_radius'] # r
        self.predict_number = kwargs['predict_number'] # p
        self.k_rotation = kwargs['k_rotation']
        self.k_along_path = kwargs['k_along_path']
        # self.distance_threshold = kwargs['distance_threshold']
        # self.angle_threshold = kwargs['angle_threshold']

        # 滑动窗口队列
        self.q_passed_idx = self.along_path_radius # r (fixed, 相对滑动窗口队列)
        self.q_size = self.along_path_radius + 1 + max(self.along_path_radius, self.predict_number)
        """
            0   1   2   3   4   5   6   7   8   9
            x   x   D0  D1  D2  D3  D4  D5  D6  D7
           |- r -|  ^  |- r -|
                    r  |----------- p -----------|
        """
        self.q = ListQueue(size = self.q_size) # (image, odom)

        # 运行时
        self.passed_goal = -1 # i
        self.T_0_odomA = None
        self.T_0_odomB = None
    
    def pass_to_next_goal(self):
        # loader 读完后不断发送 None, 此处不断入队, 直到 q_passed_idx + 1 项也为 None 时即结束
        self.q.push(self.io['record'].read(block = True))
        self.passed_goal += 1
        self.T_0_odomA = self.T_0_odomB
        T_odomA_odomB = np.dot(
            t3d_ext.einv(self.q[self.q_passed_idx][1]),
            self.q[self.q_passed_idx + 1][1]
        ) # T_odomA_odomB = T_{rec_r}_{rec_(r+1)} = inv(T_0_{rec_r}) * T_0_{rec_(r+1)}
        self.T_0_odomB = np.dot(
            self.T_0_odomA,
            T_odomA_odomB
        )
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)

        # 检查接口
        while not ros.is_shutdown():
            if 'processed_image' not in self.io or 'odom' not in self.io or 'actuator_command' not in self.io or 'record' not in self.io:
                time.sleep(0.1)
                continue
            break

        # 凑满 q_size 个数据, 前 along_path_radius 个为 None, 其余阻塞读取
        self.q.q = [None] * self.along_path_radius
        for i in range(self.q_size - self.along_path_radius):
            self.q.push(self.io['record'].read(block = True))
        
        # 获取最初的 odom 作为 T_0_odomB (pass_to_next_goal 中赋值给 T_0_odomA)
        self.T_0_odomB = self.io['odom'].read(block = True)
        self.pass_to_next_goal() # -1 to 0; 更新 T_0_odomA, T_0_odomB

        # 主循环
        while not ros.is_shutdown():
            # 结束
            if self.q.q[self.q_passed_idx + 1] is None:
                print('Finished.')
                break

            # Test
            time.sleep(0.05)
            l = [x[1] for x in self.q.q if x is not None]
            ros.publish_topic('/test_path', t3d_ext.es2P(l, frame_id = 'odom'))
            self.pass_to_next_goal()
            ros.publish_topic('/a', t3d_ext.e2PS(self.T_0_odomA, frame_id = 'odom'))
            ros.publish_topic('/b', t3d_ext.e2PS(self.T_0_odomB, frame_id = 'odom'))

            # 读取 processed_image 和 odom
            if self.io['processed_image'].poll() and self.io['odom'].poll():
                image = self.io['processed_image'].read()
                odom = self.io['odom'].read()

                # TODO

