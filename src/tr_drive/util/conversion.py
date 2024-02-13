import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
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
        if other == 0:
            raise ValueError("Division by zero")
        return Vec3(self.x / other, self.y / other, self.z / other)
    
    def __floordiv__(self, other: float):
        if other == 0:
            raise ValueError("Division by zero")
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
        if other == 0:
            raise ValueError("Division by zero")
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
    
    def to_euler(self): # TODO: validate.
        # RPY; ZYX order
        x, y, z, w = self.normalize().to_list()
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = np.arcsin(2 * (w * y - z * x))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        return np.array([roll, pitch, yaw])
    
    def to_rotation_matrix(self): # TODO: validate.
        x, y, z, w = self.normalize().to_list()
        return Mat3([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
        ])
    
    def to_rotation_vector(self): # TODO: validate.
        x, y, z, w = self.normalize().to_list()
        theta = 2 * np.arccos(w)
        n = Vec3([x, y, z]).normalize()
        return n * theta
    
    def validate(self):
        return abs(self.norm() - 1) < 1e-6

    def inverse(self):
        return self.conjugate() / self.norm() ** 2

    def rotate(self, v: Vec3):
        self_normalized = self.normalize()
        return Vec3((self_normalized * Quat(v.to_list() + [0]) * self_normalized.inverse()).to_list()[:3])
    
    def conjugate(self):
        return Quat(-self.x, -self.y, -self.z, self.w)
    
    def norm(self):
        return np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2)
    
    def normalize(self):
        return self / self.norm()


class Frame:
    def __init__(self, *args):
        if len(args) == 0:
            self.translation = Vec3()
            self.quaternion = Quat()
        elif len(args) == 1 and isinstance(args[0], Pose):
            frame = Frame.from_Pose(args[0])
            self.translation, self.quaternion = frame.translation, frame.quaternion
        elif len(args) == 1 and isinstance(args[0], Odometry):
            frame = Frame.from_Odometry(args[0])
            self.translation, self.quaternion = frame.translation, frame.quaternion
        elif len(args) == 2: # and isinstance(args[0], Vec3) and isinstance(args[1], Quat):
            self.translation, self.quaternion = args
        else:
            raise ValueError("Invalid arguments")
    
    def __mul__(self, other: 'Frame'):
        return self.transform(other)
    
    def __str__(self):
        return f'Frame[t = {self.translation}, q = {self.quaternion}]'
    
    @property
    def t(self):
        return self.translation
    
    @property
    def q(self):
        return self.quaternion
    
    @property
    def I(self):
        return self.inverse()

    @staticmethod
    def from_Pose(msg: Pose):
        res = Frame()
        res.translation = Vec3([msg.position.x, msg.position.y, msg.position.z])
        res.quaternion = Quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        if not res.quaternion.validate():
            # raise ValueError("Invalid quaternion")
            res.quaternion = Quat(0, 0, 0, 1)
        return res

    @staticmethod
    def from_PoseStamped(msg: PoseStamped):
        return Frame.from_Pose(msg.pose)
    
    @staticmethod
    def from_Odometry(msg: Odometry):
        return Frame.from_Pose(msg.pose.pose)

    @staticmethod
    def from_dict(d: dict):
        return Frame(Vec3(d['translation']), Quat(d['quaternion']))
    
    def to_Pose(self):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = self.translation.to_list()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = self.quaternion.to_list()
        return msg
    
    def to_PoseStamped(self, frame_id = ''):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose = self.to_Pose()
        return msg
    
    def to_Odometry(self, frame_id = ''):
        msg = Odometry()
        msg.header.frame_id = frame_id # TODO
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.translation.to_list()
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = self.quaternion.to_list()
        return msg

    def to_dict(self):
        return {
            'translation': self.translation.to_list(),
            'quaternion': self.quaternion.to_list()
        }
    
    def inverse(self):
        q_inv = self.quaternion.I
        return Frame(-q_inv.rotate(self.translation), q_inv)
    
    def transform(self, other: 'Frame'):
        return Frame(self.translation + self.quaternion.rotate(other.translation), self.quaternion * other.quaternion)
    
    def yaw_difference(self, other: 'Frame'):
        yaw_0 = self.quaternion.Euler[2]
        yaw_1 = other.quaternion.Euler[2]
        delta_yaw = abs(yaw_0 - yaw_1)
        if delta_yaw > np.pi:
            delta_yaw = np.pi * 2 - delta_yaw
        return delta_yaw
    
    def translation_difference(self, other: 'Frame'):
        return (other.I * self).translation.norm()

