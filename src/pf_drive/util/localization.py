import numpy as np

from pf_drive.util.image import DigitalImage


class LocationFeature:
    def __init__(self, *args, **kwargs): pass

    @staticmethod
    def from_file(file_path: str): pass

    def to_file(self, file_path: str): pass

    def distance(self, other: 'LocationFeature'): pass


class VisualPlaceFeatureVector(LocationFeature):
    def __init__(self, arg: DigitalImage | list | np.ndarray = None):
        if isinstance(arg, DigitalImage):
            self.v = np.array([0]) # TODO: generate feature vector from img
        elif isinstance(arg, list) or isinstance(arg, np.ndarray):
            self.v = np.array(arg)
        else:
            raise ValueError(f'Invalid argument type: {type(arg)}')
    
    @staticmethod
    def from_file(file_path: str):
        with open(file_path, 'r') as f:
            return VisualPlaceFeatureVector([float(x) for x in f.read().split(',')])

    def to_file(self, file_path: str):
        with open(file_path, 'w') as f:
            f.write(','.join(map(str, self.v)))
    
    def distance(self, other: 'VisualPlaceFeatureVector'):
        return np.dot(self.v, other.v) / (np.linalg.norm(self.v) * np.linalg.norm(other.v))

