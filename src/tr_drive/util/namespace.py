import os
import time

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path


class DictRegulator:
    def __init__(self, d):
        for key, value in d.items():
            if isinstance(value, dict):
                setattr(self, key, DictRegulator(value))
            else:
                setattr(self, key, value)
    
    def add(self, key, value):
        setattr(self, key, value)
    
    def remove(self, key):
        delattr(self, key)
    
    def __contains__(self, key):
        return hasattr(self, key)
    
    def to_dict(self):
        res = {}
        for key, value in self.__dict__.items():
            if isinstance(value, DictRegulator):
                res[key] = value.to_dict()
            else:
                res[key] = value
        return res


def generate_time_str():
    time.strftime('%Y-%m-%d_%H:%M:%S')


def recursive_dict_update(d: dict, u: dict):
    for k, v in u.items():
        if isinstance(v, dict):
            d[k] = recursive_dict_update(d.get(k, {}), v)
        else:
            d[k] = v
    return d


def type_from_str(type: str):
    if type == 'Pose':
        return Pose
    elif type == 'PoseStamped':
        return PoseStamped
    elif type == 'PoseWithCovariance':
        return PoseWithCovariance
    elif type == 'PoseWithCovarianceStamped':
        return PoseWithCovarianceStamped
    elif type == 'Odometry':
        return Odometry
    elif type == 'Path':
        return Path
    else:
        raise ValueError('Unsupported type string')


def get_filename_number(filename):
    try:
        res = int((filename + '.').split('.')[0])
    except ValueError:
        res = -1
    return res


def get_sorted_file_list(folder: str):
    return sorted(os.listdir(folder), key = lambda x: get_filename_number(x))

