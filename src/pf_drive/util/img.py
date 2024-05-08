import time

import numpy as np

import cv2
from cv_bridge import CvBridge

from scipy import signal


"""
    Conversion between ROS message (Image) and np.ndarray
"""
bridge = CvBridge()

def np_to_Image(img, encoding = 'passthrough'):
    return bridge.cv2_to_imgmsg(img, encoding = encoding)

def Image_to_np(msg, desired_encoding = 'passthrough'):
    return bridge.imgmsg_to_cv2(msg, desired_encoding = desired_encoding)


"""
    Draw
"""
# def 


"""
    Image processing
"""
def patch_normalize(img, patch_size):
    patch_radius = (patch_size - 1) // 2
    r, c = img.shape
    
    img_pad = np.pad(np.float64(img), patch_radius, 'constant', constant_values = np.nan)
    
    kernels = np.lib.stride_tricks.as_strided(img_pad, (r, c, patch_size, patch_size), img_pad.strides + img_pad.strides).reshape(r, c, -1)
    kernel_mean = np.nanmean(kernels, axis = 2).reshape(r, c)
    kernel_std = np.nanstd(kernels, axis = 2).reshape(r, c)
    with np.errstate(divide = 'ignore', invalid = 'ignore'):
        res = (img - kernel_mean) / kernel_std
    res[np.isnan(res)] = 0.0
    np.clip(res, -1.0, 1.0, out = res)
    
    return np.uint8((res + 1.0) / 2 * 255)

def horizontal_cumsum(image, width):
	cumsum = image.sum(axis = 0).cumsum()
	return cumsum[(width - 1):] - cumsum[:(1 - width)]

def NCC_horizontal_scan(img, img_ref):
    img = np.pad(img.astype(float) / 256.0, ((0, ), (int(img.shape[1] / 2), )), mode = 'constant', constant_values = 0)
    img_ref = img_ref.astype(float) / 256.0

    img_delta = img - img.mean()
    img_ref_delta = img_ref - img_ref.mean()
    numerator = signal.correlate2d(img_delta, img_ref_delta, mode = 'valid')

    I_hat = horizontal_cumsum(img, img_ref.shape[1])
    I_squared = horizontal_cumsum(img ** 2, img_ref.shape[1])

    denominator = np.sqrt((I_squared - I_hat ** 2 / img_ref.size) * (img_ref_delta ** 2).sum())
    
    with np.errstate(divide = 'raise', invalid = 'raise'):
        try:
            res = (numerator / denominator)[0]
        except Exception:
            res = np.zeros(img.shape[1] - img_ref.shape[1] + 1)
            res[len(res) // 2] = 1
    return res

def NCC_horizontal_match(img, img_ref):
    values = NCC_horizontal_scan(img, img_ref)
    offset = np.argmax(values)
    return int(offset - (len(values) - 1) / 2), values[offset], values[int((len(values) - 1) / 2)]

# propective_offset = 70 # positive: counter-clockwise (objects move right from img_ref to img)

# img_ref = np.random.rand(50, 150)
# if propective_offset > 0:
#     img = np.concatenate((np.zeros((50, propective_offset)), img_ref[:,:(150 - propective_offset)]), axis = 1)
# else:
#     img = np.concatenate((img_ref[:, -propective_offset:], np.zeros((50, -propective_offset))), axis = 1)

# t = time.time()
# for i in range(5):
#     l = NCC_horizontal_match(img, img_ref)
# print(time.time() - t)
# print((int(l[0]), l[1], l[2]))

