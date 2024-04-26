import os

import numpy as np
import cv2

from sensor_msgs.msg import Image

from pf_drive.util.namespace import get_sorted_file_list


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
        elif len(args) == 3 and isinstance(args[0], int) and isinstance(args[1], int) and isinstance(args[2], int):
            self.width, self.height, self.channel = args
            self.data = np.zeros((self.height, self.width, self.channel), dtype = np.uint8)
        else:
            raise ValueError("Invalid arguments")
    
    def copy(self):
        res = DigitalImage(self.width, self.height, self.channel)
        res.data = self.data.copy()
        return res
    
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
    
    @staticmethod
    def from_file(file_path: str):
        data = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)
        return DigitalImage(data)
    
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
    
    def to_file(self, file_path: str): # jpeg
        if self.channel == 1:
            cv2.imwrite(file_path, self.data)
        elif self.channel == 3:
            # cv2.imwrite(path, self.data)
            pass # TODO
        elif self.channel == 4:
            cv2.imwrite(file_path, self.data[:, :, [2, 1, 0, 3]])
        else:
            raise ValueError("Unsupported channel")
    
    def to_grayscale(self):
        if self.channel == 1:
            return self
        elif self.channel == 3:
            return DigitalImage(np.expand_dims(np.dot(self.data, [0.299, 0.587, 0.114]).astype(np.uint8), axis = 2))
        elif self.channel == 4:
            return DigitalImage(np.expand_dims(np.dot(self.data[:, :, :3], [0.299, 0.587, 0.114]).astype(np.uint8), axis = 2))
        else:
            raise ValueError("Unsupported channel")
    
    def to_rgb(self):
        data = self.data
        if self.channel == 1:
            data = np.concatenate([data, data, data], axis = -1)
        elif self.channel == 3:
            pass
        elif self.channel == 4:
            data = data[:, :, [2, 1, 0]]
        else:
            raise ValueError("Unsupported channel")
        return DigitalImage(data)
    
    def interpolate(self, width: int, height: int):
        return DigitalImage(cv2.resize(self.data, (width, height), interpolation = cv2.INTER_AREA))


class DigitalImageList:
    # Python 貌似无类似模板类的东西, 暂且写两遍类似的东西, 详细注释见 geometry 中的 FrameList.
    def __init__(self, imgs: list = [], bound_folder = None):
        assert all(isinstance(img, DigitalImage) for img in imgs)
        
        self.data = list(imgs)
        self.bind_folder(bound_folder)
    
    def __getitem__(self, index):
        assert isinstance(index, int)
        if index < 0:
            index += len(self)
        if self.is_folder_bound():
            return DigitalImage.from_file(self.bound_folder + '/' + str(index) + '.jpg')
        else:
            return self.data[index]

    def __setitem__(self, index, value):
        assert isinstance(index, int) and isinstance(value, DigitalImage)
        if index < 0:
            index += len(self)
        if self.is_folder_bound():
            value.to_file(self.bound_folder + '/' + str(index) + '.jpg')
        else:
            self.data[index] = value

    def __len__(self):
        if self.is_folder_bound():
            return sum([1 for filename in os.listdir(self.bound_folder) if DigitalImageList.is_filename_valid(filename)])
        else:
            return len(self.data)
    
    def append(self, img: DigitalImage):
        if self.is_folder_bound():
            self.__setitem__(len(self), img)
        else:
            self.data.append(img)

    def copy(self):
        return DigitalImageList([img.copy() for img in self.data])
    
    @staticmethod
    def is_filename_valid(filename: str):
        return filename.endswith('.jpg') # TODO
    
    @staticmethod
    def from_file(folder_path: str, ifio: bool = False, allow_not_existed: bool = False):
        if ifio:
            return DigitalImageList(bound_folder = folder_path)
        else:
            if not os.path.exists(folder_path):
                if allow_not_existed:
                    return DigitalImageList([])
                raise FileNotFoundError("Folder not found.")
            imgs = []
            for filename in get_sorted_file_list(folder_path):
                if DigitalImageList.is_filename_valid(filename):
                    imgs.append(DigitalImage.from_file(folder_path + '/' + filename))
            return DigitalImageList(imgs)
    
    def to_file(self, folder_path: str):
        if not self.is_folder_bound():
            os.makedirs(folder_path, exist_ok = True)
            for i, img in enumerate(self.data):
                img.to_file(folder_path + '/' + str(i) + '.jpg')

    def is_folder_bound(self):
        return self.bound_folder is not None
    
    def bind_folder(self, folder_path: str, clear_memory_data: bool = True):
        self.bound_folder = folder_path
        if folder_path is not None:
            os.makedirs(folder_path, exist_ok = True)
            if clear_memory_data:
                self.data.clear()
    
    def unbind_folder(self, load_data_into_memory: bool = False):
        if self.is_folder_bound():
            if load_data_into_memory:
                self.data = DigitalImageList.from_file(self.bound_folder).data
            self.bound_folder = None

    def clear(self):
        if self.is_folder_bound():
            for filename in os.listdir(self.bound_folder):
                if DigitalImageList.is_filename_valid(filename):
                    os.remove(self.bound_folder + '/' + filename)
        else:
            self.data.clear()


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
    def best_match_offset(image: DigitalImage, image_ref: DigitalImage, range_radius: int): # r + 1 + r
        offset_list = list(range(-range_radius, range_radius + 1))
        values = ImageProcessor.horizontal_NCC(image, image_ref, offset_list)
        idx = np.argmax(values)
        return offset_list[idx], values[idx]

    def draw_line(image: DigitalImage, x0: int, y0: int, x1: int, y1: int, color: list, thickness: int = 1):
        assert image.channel == len(color)
        res = image.copy()
        res.data = cv2.line(res.data, (x0, y0), (x1, y1), color, thickness)
        return res

