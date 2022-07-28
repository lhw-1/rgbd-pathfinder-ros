import cv2
import numpy as np
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from ipm import IPM
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def convert_to_monochrome(traversable_seg):
    # traversable_seg is a numpy array corresponding to the panoptic segmentation
    # Traversable areas have positive integers as values, and Non-traversable areas have -1
    traversable_seg_mnc = np.zeros((len(traversable_seg), len(traversable_seg[0]), 3))

    for i in range(len(traversable_seg)):
        for j in range(len(traversable_seg[0])):
            if not traversable_seg[i][j] == -1:
                traversable_seg_mnc[i][j] = [255, 255, 255]
    
    return traversable_seg_mnc

class _DictObjHolder(object):
    def __init__(self, dct):
        self.dct = dct

    def __getattr__(self, name):
        return self.dct[name]

def convert_to_bev(traversable_seg_mnc):
    camera_info = _DictObjHolder({
        "f_x": 919,         # focal length x
        "f_y": 920,         # focal length y
        "u_x": 647,             # optical center x
        "u_y": 369,             # optical center y
        "camera_height": 800,  # camera height in `mm`
        "pitch": 105,           # rotation degree around x
        "yaw": 0                # rotation degree around y
    })
    ipm_info = _DictObjHolder({
       "input_width": 1280,
       "input_height": 720,
       "out_width": 1280,
       "out_height": 720,
       "left": 0,
       "right": 1280,
       "top": 0,
       "bottom": 720
    }) 

    # path = "frame1000.jpg"
    # img = load_img(path)
    # img = img_to_array(img)
    img = traversable_seg_mnc
    if len(img.shape) == 3:
        # img = np.dot(img, [0.299, 0.587, 0.114])
        img = np.dot(img, [0.333, 0.333, 0.333])

    print(img.shape)
    ipm = IPM(camera_info, ipm_info)
    out_img = ipm(img)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    ax.imshow(img)
    ax = fig.add_subplot(212)
    ax.imshow(out_img)

    uv = ipm.xy2uv(np.array([[0.71*1e3], [4.14*1e3]]))
    print('uv ', uv)
    uv = ipm.xy2uv(np.array([[0.88*1e3], [6.86*1e3]]))
    print('uv ', uv)
    plt.savefig("ipm.png")

    return out_img
