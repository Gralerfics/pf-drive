import numpy as np
import cv2

from sensor_msgs.msg import Image


class DigitalImage:
    def __init__(self, *args):
        if len(args) == 0:
            self.width, self.height, self.channel = 0, 0, 0
            self.data = None
        elif len(args) == 1 and isinstance(args[0], np.ndarray):
            # TODO: assert shape.
            if len(args[0].shape) == 2:
                self.width, self.height, self.channel = args[0].shape[1], args[0].shape[0], 1
                self.data = np.expand_dims(args[0], axis = 2)
            else:
                self.height, self.width, self.channel = args[0].shape
                self.data = args[0]
        elif len(args) == 1 and isinstance(args[0], Image):
            img = self.from_Image(args[0])
            self.width, self.height, self.channel = img.width, img.height, img.channel
            self.data = img.data
        else:
            raise ValueError("Invalid arguments")
    
    @staticmethod
    def from_Image(msg: Image):
        img = DigitalImage()
        img.width = msg.width
        img.height = msg.height
        # TODO: take msg.is_bigendian into account.
        if msg.encoding == 'rgb8':
            img.channel = 3
            img.data = np.frombuffer(msg.data, dtype=np.uint8).reshape((img.height, img.width, img.channel))
        elif msg.encoding == 'bgr8':
            img.channel = 3
            img.data = np.frombuffer(msg.data, dtype=np.uint8).reshape((img.height, img.width, img.channel))[:, :, [2, 1, 0]]
        elif msg.encoding == 'mono8':
            img.channel = 1
            img.data = np.frombuffer(msg.data, dtype=np.uint8).reshape((img.height, img.width, 1))
        elif msg.encoding == 'mono16':
            img.channel = 1
            img.data = np.frombuffer(msg.data, dtype=np.uint16).reshape((img.height, img.width, 1))
        elif msg.encoding == 'rgba8':
            img.channel = 4
            img.data = np.frombuffer(msg.data, dtype=np.uint8).reshape((img.height, img.width, img.channel))
        elif msg.encoding == 'bgra8':
            img.channel = 4
            img.data = np.frombuffer(msg.data, dtype=np.uint8).reshape((img.height, img.width, img.channel))[:, :, [2, 1, 0, 3]]
        elif msg.encoding == '32FC1':
            img.channel = 1
            img.data = np.frombuffer(msg.data, dtype=np.float32).reshape((img.height, img.width, 1))
        else:
            raise ValueError("Unsupported encoding")
        return img
    
    def to_Image(self, encoding = 'rgb8'):
        msg = Image()
        msg.width = self.width
        msg.height = self.height
        msg.step = self.channel * self.width
        msg.encoding = encoding
        msg.is_bigendian = 0 # TODO: take this into account.
        if encoding == 'rgb8' and self.channel == 3:
            data = self.data.reshape(-1).tolist()
        elif encoding == 'bgr8' and self.channel == 3:
            data = self.data[:, :, [2, 1, 0]].reshape(-1).tolist()
        elif encoding == 'rgba8' and self.channel == 4:
            data = self.data.reshape(-1).tolist()
        elif encoding == 'bgra8' and self.channel == 4:
            data = self.data[:, :, [2, 1, 0, 3]].reshape(-1).tolist()
        elif encoding in ['mono8', 'mono16', '32FC1'] and self.channel == 1:
            data = self.data.reshape(-1).tolist()
        else:
            raise ValueError("Invalid conversion")
        msg.data = bytes(data)
        return msg
    
    def grayscale(self):
        if self.channel == 1:
            return self
        elif self.channel == 3:
            return DigitalImage(np.expand_dims(np.dot(self.data, [0.299, 0.587, 0.114]).astype(np.uint8), axis = 2))
        elif self.channel == 4: # TODO
            return DigitalImage(np.expand_dims(np.dot(self.data[:, :, :3], [0.299, 0.587, 0.114]).astype(np.uint8), axis = 2))
        else:
            raise ValueError("Unsupported channel")
    
    def interpolate(self, width: int, height: int):
        return DigitalImage(cv2.resize(self.data, (width, height), interpolation = cv2.INTER_AREA))


class ImageProcessor:
    @staticmethod
    def kernel_normalize(image, patch_size: int):
        assert image.channel == 1
        patch_radius = (patch_size - 1) // 2
        r, c = image.height, image.width
        
        data_squeeze = np.squeeze(image.data, axis = 2)
        data_pad = np.pad(np.float64(data_squeeze), patch_radius, 'constant', constant_values = np.nan)
        
        kernels = np.lib.stride_tricks.as_strided(data_pad, (r, c, patch_size, patch_size), data_pad.strides + data_pad.strides).reshape(r, c, -1)
        kernel_mean = np.nanmean(kernels, axis = 2).reshape(r, c)
        kernel_std = np.nanstd(kernels, axis = 2).reshape(r, c)
        with np.errstate(divide = 'ignore', invalid = 'ignore'):
            res = (data_squeeze - kernel_mean) / kernel_std
        res[np.isnan(res)] = 0.0
        np.clip(res, -1.0, 1.0, out = res)
        
        return DigitalImage(np.uint8((res + 1.0) / 2 * 255))
    
    @staticmethod
    def normalized_cross_correlation(image, image_ref, offset_list): # image_ref --offset_list[i]-> image ==> value[i]
        assert image.channel == 1 and image_ref.channel == 1
        r, c = image.height, image.width
        r_ref, c_ref = image_ref.height, image_ref.width
        assert r == r_ref and c == c_ref
        
        

