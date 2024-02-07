import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class Vec3:
    def __init__(self, *args):
        if len(args) == 0:
            self.x, self.y, self.z = 0.0, 0.0, 0.0
        elif len(args) == 1 and (isinstance(args[0], list) or isinstance(args[0], np.ndarray)):
            self.x, self.y, self.z = args[0]
        elif len(args) == 3:
            self.x, self.y, self.z = args
        else:
            raise ValueError("Invalid arguments")
    
    def __add__(self, other: 'Vec3'):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Vec3'):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)
    
    def __mul__(self, other: float):
        return Vec3(self.x * other, self.y * other, self.z * other)
    
    def __truediv__(self, other: float):
        return Vec3(self.x / other, self.y / other, self.z / other)
    
    def __floordiv__(self, other: float):
        return Vec3(self.x // other, self.y // other, self.z // other)
    
    def __str__(self):
        return f'Vector3[{self.x}, {self.y}, {self.z}]'
    
    def to_list(self):
        return [self.x, self.y, self.z]
    
    def to_np(self):
        return np.array([self.x, self.y, self.z])
    
    def dot(self, other: 'Vec3'):
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other: 'Vec3'):
        return Vec3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
    
    def norm(self):
        return np.sqrt(self.dot(self))
    
    def normalize(self):
        return self / self.norm()


class Mat3:
    def __init__(self, *args):
        if len(args) == 0:
            self.data = np.eye(3)
        elif len(args) == 1 and (isinstance(args[0], list) or isinstance(args[0], np.ndarray)):
            # TODO: assert shape
            self.data = np.array(args[0])
        elif len(args) == 9:
            self.data = np.array(args).reshape(3, 3)
        else:
            raise ValueError("Invalid arguments")
    
    def __mul__(self, other):
        if isinstance(other, Mat3):
            return Mat3(self.data @ other.data)
        elif isinstance(other, Vec3):
            return Vec3(self.data @ other.to_np())
        else:
            raise TypeError("Invalid type")
    
    def __str__(self):
        return f'Matrix3{self.to_list()}'
    
    def to_list(self):
        return self.data.flatten().tolist()
    
    def to_np(self):
        return self.data


class Quat:
    def __init__(self, *args):
        if len(args) == 0:
            self.x, self.y, self.z, self.w = 0.0, 0.0, 0.0, 1.0
        elif len(args) == 1 and (isinstance(args[0], list) or isinstance(args[0], np.ndarray)):
            self.x, self.y, self.z, self.w = args[0]
        elif len(args) == 4:
            self.x, self.y, self.z, self.w = args
        else:
            raise ValueError("Invalid arguments")
    
    def __mul__(self, other: 'Quat'):
        return Quat(
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        )
    
    def __truediv__(self, other: float):
        return Quat(self.x / other, self.y / other, self.z / other, self.w / other)
    
    def __str__(self):
        return f'Quaternion[x = {self.x}, y = {self.y}, z = {self.z}, w = {self.w}]'
    
    @property
    def I(self):
        return self.inverse()
    
    @property
    def Euler(self):
        return self.to_euler()
    
    @property
    def R(self):
        return self.to_rotation_matrix()
    
    @property
    def V(self):
        return self.to_rotation_vector()
    
    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float):
        # TODO
        pass
    
    @staticmethod
    def from_rotation_matrix(R: Mat3):
        # TODO
        pass
    
    @staticmethod
    def from_rotation_vector(v: Vec3):
        theta = v.norm()
        n = v.normalize()
        return Quat(n.x * np.sin(theta / 2), n.y * np.sin(theta / 2), n.z * np.sin(theta / 2), np.cos(theta / 2))
    
    def to_list(self):
        return [self.x, self.y, self.z, self.w]
    
    def to_np(self):
        return np.array([self.x, self.y, self.z, self.w])
    
    def to_euler(self):
        # RPY; ZYX order
        roll = np.arctan2(2 * (self.w * self.x + self.y * self.z), 1 - 2 * (self.x ** 2 + self.y ** 2))
        pitch = np.arcsin(2 * (self.w * self.y - self.z * self.x))
        yaw = np.arctan2(2 * (self.w * self.z + self.x * self.y), 1 - 2 * (self.y ** 2 + self.z ** 2))
        return np.array([roll, pitch, yaw])
    
    def to_rotation_matrix(self):
        x, y, z, w = self.to_list()
        return Mat3([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
        ])
    
    def to_rotation_vector(self):
        theta = 2 * np.arccos(self.w)
        n = Vec3([self.x, self.y, self.z]).normalize()
        return n * theta

    def inverse(self):
        return self.conjugate() / self.norm() ** 2

    def rotate(self, v: Vec3):
        return Vec3((self * Quat(v.to_list() + [0]) * self.inverse()).to_list()[:3])
    
    def conjugate(self):
        return Quat(-self.x, -self.y, -self.z, self.w)
    
    def norm(self):
        return np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2)
    
    def normalize(self):
        return self / self.norm()


class Frame:
    def __init__(self, *args):
        if len(args) == 0:
            self.t = Vec3()
            self.q = Quat()
        elif len(args) == 1 and isinstance(args[0], Pose):
            frame = Frame.from_Pose(args[0])
            self.t, self.q = frame.t, frame.q
        elif len(args) == 1 and isinstance(args[0], Odometry):
            frame = Frame.from_Odometry(args[0])
            self.t, self.q = frame.t, frame.q
        elif len(args) == 2: # and isinstance(args[0], Vec3) and isinstance(args[1], Quat):
            self.t, self.q = args
        else:
            raise ValueError("Invalid arguments")
    
    def __mul__(self, other: 'Frame'):
        return self.transform(other)
    
    def __str__(self):
        return f'Frame[t = {self.t}, q = {self.q}]'
    
    @property
    def I(self):
        return self.inverse()

    @staticmethod
    def from_Pose(msg: Pose):
        res = Frame()
        res.t = Vec3([msg.position.x, msg.position.y, msg.position.z])
        res.q = Quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        return res
    
    @staticmethod
    def from_Odometry(msg: Odometry):
        return Frame.from_Pose(msg.pose.pose)
    
    def to_Pose(self):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = self.t.to_list()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = self.q.to_list()
        return msg
    
    def to_Odometry(self):
        msg = Odometry()
        # TODO: header
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.t.to_list()
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = self.q.to_list()
        return msg
    
    def inverse(self):
        q_inv = self.q.I
        return Frame(-q_inv.rotate(self.t), q_inv)
    
    def transform(self, other: 'Frame'):
        return Frame(self.t + self.q.rotate(other.t), self.q * other.q)
