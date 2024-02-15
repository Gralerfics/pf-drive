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
            if args[0].ndim == 2:
                self.width, self.height, self.channel = args[0].shape[1], args[0].shape[0], 1
                self.data = np.expand_dims(args[0], axis = 2)
            elif args[0].ndim == 3:
                self.height, self.width, self.channel = args[0].shape
                self.data = args[0]
            else:
                raise ValueError("Invalid array shape")
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

    def to_imgui_texture(self):
        data = self.data
        if self.channel == 1:
            data = np.concatenate([data, data, data, 255 * np.ones(data.shape)], axis = -1)
        elif self.channel == 3:
            data = np.concatenate([data, 255 * np.ones((*data.shape[:-1], 1))], axis = -1)
        elif self.channel == 4:
            pass
        else:
            raise ValueError("Unsupported channel")
        return (data.reshape(-1).astype(float) / 255).tolist()
    
    def to_jpg(self, path):
        if self.channel == 1:
            cv2.imwrite(path, self.data)
        elif self.channel == 3:
            # cv2.imwrite(path, self.data)
            pass # TODO
        elif self.channel == 4:
            cv2.imwrite(path, self.data[:, :, [2, 1, 0, 3]])
        else:
            raise ValueError("Unsupported channel")
    
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
        
        data = np.squeeze(image.data, axis = 2)
        data_pad = np.pad(np.float64(data), patch_radius, 'constant', constant_values = np.nan)
        
        kernels = np.lib.stride_tricks.as_strided(data_pad, (r, c, patch_size, patch_size), data_pad.strides + data_pad.strides).reshape(r, c, -1)
        kernel_mean = np.nanmean(kernels, axis = 2).reshape(r, c)
        kernel_std = np.nanstd(kernels, axis = 2).reshape(r, c)
        with np.errstate(divide = 'ignore', invalid = 'ignore'):
            res = (data - kernel_mean) / kernel_std
        res[np.isnan(res)] = 0.0
        np.clip(res, -1.0, 1.0, out = res)
        
        return DigitalImage(np.uint8((res + 1.0) / 2 * 255))
    
    @staticmethod
    def NCC(data: np.ndarray, data_ref: np.ndarray): # -> [-1, 1]
        assert data.shape == data_ref.shape and data.ndim == 2
        
        mean = np.mean(data)
        mean_ref = np.mean(data_ref)
        std = np.std(data)
        std_ref = np.std(data_ref)
        
        return np.sum((data - mean) * (data_ref - mean_ref)) / (data.shape[0] * data.shape[1] * std * std_ref)
    
    @staticmethod
    def horizontal_NCC(image, image_ref, offset_list): # image_ref --offset_list[i]-> image ==> value[i]
        assert image.channel == 1 and image_ref.channel == 1 and image.height == image_ref.height and image.width == image_ref.width
        assert not (np.any(np.array(offset_list) <= -image.width) or np.any(np.array(offset_list) >= image.width))
        
        r, c = image.height, image.width
        
        data = np.squeeze(image.data, axis = 2)
        data_ref = np.squeeze(image_ref.data, axis = 2)
        data_shifted = [data[:, -min(0, offset) : c - max(0, offset)] for offset in offset_list]
        data_ref_shifted = [data_ref[:, max(0, offset) : c + min(0, offset)] for offset in offset_list]
        
        res = np.zeros(len(offset_list))
        for i in range(len(offset_list)):
            res[i] = ImageProcessor.NCC(data_shifted[i], data_ref_shifted[i])
        return res.tolist()
    
    @staticmethod
    def best_match_offset(image, image_ref, range_radius): # r + 1 + r
        offset_list = list(range(-range_radius, range_radius + 1))
        values = ImageProcessor.horizontal_NCC(image, image_ref, offset_list)
        idx = np.argmax(values)
        return offset_list[idx], values[idx]

