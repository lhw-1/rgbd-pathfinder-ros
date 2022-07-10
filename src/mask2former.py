# Copyright (c) Facebook, Inc. and its affiliates.
# Copied from: https://github.com/facebookresearch/Mask2Former/blob/main/demo/demo.py
import argparse
import glob
import multiprocessing as mp
import os

# fmt: off
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))
# fmt: on

import tempfile
import time
import warnings

import cv2
import numpy as np
import torch
import tqdm

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.projects.deeplab import add_deeplab_config
from detectron2.utils.logger import setup_logger

from config import add_maskformer2_config
from predictor import VisualizationDemo


# constants
WINDOW_NAME = "mask2former demo"


def setup_cfg(model, configs):
    # load config from file and command-line arguments
    cfg = get_cfg()
    add_deeplab_config(cfg)
    add_maskformer2_config(cfg)
    cfg.merge_from_file(configs)
    cfg.merge_from_list(['MODEL.WEIGHTS', model])
    cfg.freeze()
    return cfg


# TODO: Include webcam / video methods
def run_segmentation(model, configs, inputs=[], output_dir=''):
    mp.set_start_method("spawn", force=True)
    args = [model, configs, inputs, output_dir]
    setup_logger(name="fvcore")
    logger = setup_logger()
    logger.info("Arguments: " + str(args))
    cfg = setup_cfg(model, configs)
    vis_demo = VisualizationDemo(cfg)
    panoptic_seg = torch.Tensor([])
    segments_info = []

    if inputs:
        if len(inputs) == 1:
            inputs = glob.glob(os.path.expanduser(inputs[0]))
            assert inputs, "The input path(s) was not found"
        for path in tqdm.tqdm(inputs, disable=not inputs):
            # use PIL, to be consistent with evaluation
            img = read_image(path, format="BGR")
            start_time = time.time()
            predictions, visualized_output, panoptic_seg, segments_info = vis_demo.run_on_image(img)
            logger.info(
                "{}: {} in {:.2f}s".format(
                    path,
                    "detected {} instances".format(len(predictions["instances"]))
                    if "instances" in predictions
                    else "finished",
                    time.time() - start_time,
                )
            )
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.imshow(WINDOW_NAME, visualized_output.get_image()[:, :, ::-1])
            if cv2.waitKey(0) == 27:
                break  # esc to quit

        return panoptic_seg, segments_info