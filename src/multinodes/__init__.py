import pickle
import struct

from multiprocessing import Process, Pipe, Queue, Array, Event


class MultinodesException(Exception): pass


# TODO: RPC Cable


"""
    pipe 和 queue 接收一次后皆会弹出该数据，无法做到广播，故建议只连接两个节点 (例外情况例如多个接收节点功能是平行的);
    shared_object 适用于广播，需要指定内存大小，且锁可能降低性能;
    ...
    queue 读写时会阻塞.
"""
class Cable:
    def __init__(self, cable_type = 'queue', distributees = [], **kwargs): # distributees = [(node_obj, port_name), ...]
        self.cable_type = cable_type
        
        self.latest = kwargs.get('latest', False) # 注意 True 只适用于所有指令同类的情况, 如掺杂控制指令则可能被忽略，应当手动实现 latest
        self.size = kwargs.get('size', 100)
        self.default = kwargs.get('default', False)

        if cable_type == 'pipe': # duplex: bool, latest: bool
            self.pipe_recv, self.pipe_send = Pipe(duplex = kwargs.get('duplex', False))
        elif cable_type == 'queue': # size: int
            self.queue = Queue(maxsize = self.size)
        elif cable_type == 'shared_object': # size: int
            self.shared_obj = Array('B', size_or_initializer = 4 + self.size, lock = True)
        elif cable_type == 'event': # default: bool
            self.event = Event() # defaultly not set
            if self.default:
                self.event.set()
        elif cable_type == 'rpc': # size: int
            self.queue = Queue(maxsize = self.size)
            # TODO: 反馈队列
        else:
            raise MultinodesException('Invalid cable type.')
        
        self.distribute(distributees)
    
    def write(self, data, block = True): # block for queue (rpc)
        if self.cable_type == 'pipe':
            self.pipe_send.send(data)
        elif self.cable_type == 'queue':
            self.queue.put(data, block = block)
        elif self.cable_type == 'shared_object':
            serialized_data = pickle.dumps(data) # 序列化对象
            serialized_data_length = len(serialized_data) # 序列化后的数据长度
            if 4 + serialized_data_length > self.size: # 数据过大
                raise MultinodesException('Data too large.')
            packed_length = struct.pack('>I', serialized_data_length) # 数据长度转字节
            with self.shared_obj.get_lock():
                self.shared_obj[:(4 + serialized_data_length)] = packed_length + serialized_data # 将数据长度 (前四位) 和序列化后的数据存入共享内存
        elif self.cable_type == 'event':
            self.event.set() if bool(data) else self.event.clear()
        elif self.cable_type == 'rpc':
            self.queue.put(data, block = block) # TODO: 加上 seq 标号
    
    def feedback(self, data): # only for the sender in rpc
        if self.cable_type == 'rpc':
            pass # TODO: 取反馈队列的数据和 seq 标号

    def read(self, block = True):
        if self.cable_type == 'pipe':
            res = self.pipe_recv.recv() # 自带阻塞
            if self.latest:
                while self.pipe_recv.poll():
                    res = self.pipe_recv.recv()
            return res
        elif self.cable_type == 'queue':
            return self.queue.get(block = block)
        elif self.cable_type == 'shared_object':
            with self.shared_obj.get_lock():
                serialized_data_length = struct.unpack('>I', bytes(self.shared_obj[:4]))[0]
                serialized_data = bytes(self.shared_obj[4:(4 + serialized_data_length)])
            return pickle.loads(serialized_data)
        elif self.cable_type == 'event':
            return self.event.is_set()
        elif self.cable_type == 'rpc':
            return self.queue.get(block = block) # TODO: 分别取命令部分和 seq 标号
    
    def poll(self):
        if self.cable_type == 'pipe':
            return self.pipe_recv.poll()
        elif self.cable_type == 'queue':
            return not self.queue.empty()
        elif self.cable_type == 'shared_object':
            with self.shared_obj.get_lock():
                return struct.unpack('>I', bytes(self.shared_obj[:4]))[0] > 0
        elif self.cable_type == 'event':
            return True
        elif self.cable_type == 'rpc':
            return not self.queue.empty()
    
    def distribute(self, distributees):
        for distributee in distributees:
            if not isinstance(distributee, tuple) or len(distributee) != 2:
                raise MultinodesException('Invalid distributee.')
            distributee[0].add_cable(distributee[1], self)


class Node(Process):
    def __init__(self, name):
        super(Node, self).__init__()
        self.daemon = True

        self.name = name
        self.io = {} # cables

    def add_cable(self, port_name, cable):
        self.io[port_name] = cable
    
    def handle_rpc_once(self, rpc_cable, block = True):
        if rpc_cable.cable_type != 'rpc':
            raise MultinodesException('Not RPC cable.')
        if rpc_cable.poll() or block:
            cmd = rpc_cable.read()
            eval('self.' + cmd[0])(**cmd[1])
            return True
        return False
    
    # @override
    def run(self):
        pass

