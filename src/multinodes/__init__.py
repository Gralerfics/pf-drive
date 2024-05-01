import pickle
import struct

from multiprocessing import Process, Pipe, Queue, Value, Array


class MultinodesException(Exception): pass


"""
    Pipe 和 Queue 接收一次后皆会弹出该数据，无法做到广播，故建议只连接两个节点 (例外情况例如多个接收节点功能是平行的);
    shared_object 适用于广播，需要指定内存大小，且锁可能降低性能.
"""
class Cable:
    def __init__(self, cable_type = 'queue', distributees = [], **kwargs): # distributees = [(node_obj, port_name), ...]
        self.cable_type = cable_type
        
        self.latest = kwargs.get('latest', True)
        self.size = kwargs.get('size', 1)

        if cable_type == 'pipe': # duplex: bool, latest: bool
            self.pipe_recv, self.pipe_send = Pipe(duplex = kwargs.get('duplex', False))
        elif cable_type == 'queue': # size: int
            self.queue = Queue(maxsize = self.size)
        elif cable_type == 'shared_object': # size: int
            self.shared_obj = Array('B', size_or_initializer = 4 + self.size, lock = True)
        else:
            raise MultinodesException('Invalid cable type.')
        
        self.distribute(distributees)
    
    def write(self, data):
        if self.cable_type == 'pipe':
            self.pipe_send.send(data)
        elif self.cable_type == 'queue':
            self.queue.put(data)
        elif self.cable_type == 'shared_object':
            serialized_data = pickle.dumps(data) # 序列化对象
            serialized_data_length = len(serialized_data) # 序列化后的数据长度
            if 4 + serialized_data_length > self.size: # 数据过大
                raise MultinodesException('Data too large.')
            packed_length = struct.pack('>I', serialized_data_length) # 数据长度转字节
            with self.shared_obj.get_lock():
                self.shared_obj[:(4 + serialized_data_length)] = packed_length + serialized_data # 将数据长度 (前四位) 和序列化后的数据存入共享内存
    
    def read(self):
        if self.cable_type == 'pipe':
            # TODO: right ?
            res = self.pipe_recv.recv()
            if self.latest:
                while self.pipe_recv.poll():
                    res = self.pipe_recv.recv()
            return res
        elif self.cable_type == 'queue':
            return self.queue.get()
        elif self.cable_type == 'shared_object':
            with self.shared_obj.get_lock():
                serialized_data_length = struct.unpack('>I', bytes(self.shared_obj[:4]))[0]
                serialized_data = bytes(self.shared_obj[4:(4 + serialized_data_length)])
            return pickle.loads(serialized_data)
    
    def poll(self):
        if self.cable_type == 'pipe':
            return self.pipe_recv.poll()
        elif self.cable_type == 'queue':
            return not self.queue.empty()
        elif self.cable_type == 'shared_object':
            with self.shared_obj.get_lock():
                return struct.unpack('>I', bytes(self.shared_obj[:4]))[0] > 0
    
    def distribute(self, distributees):
        for distributee in distributees:
            if not isinstance(distributee, tuple) or len(distributee) != 2:
                raise MultinodesException('Invalid distributee.')
            distributee[0].add_cable(distributee[1], self)


class Node(Process):
    def __init__(self, name, is_shutdown_event):
        super(Node, self).__init__()
        self.name = name
        self.is_shutdown_event = is_shutdown_event
        self.io = {} # cables
    
    def is_shutdown(self):
        return self.is_shutdown_event.is_set()

    def add_cable(self, port_name, cable):
        self.io[port_name] = cable
    
    # @override
    def run(self):
        pass

