import argparse
import cv2
import glob
import multiprocessing as mp
import os
import time
import tqdm
import warnings

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger
from detectron2.projects.deeplab import add_deeplab_config

# Local Imports
from Mask2Former.mask2former import add_maskformer2_config
from predictor import VisualizationDemo

# Paths
MODEL_DIR = "../data/model_final_5c90d4.pkl"
CONFIGS_DIR = "../data/maskformer2_R50_bs16_160k.yaml"

# Parameters
WINDOW_NAME = "Mask2Former Demo"

def setup_cfg(args):
    # Load config from file and command-line arguments
    cfg = get_cfg()
    add_deeplab_config(cfg)
    add_maskformer2_config(cfg)
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    cfg.freeze()
    return cfg


def load_model():
    # Load Model for the Segmentation    
    mp.set_start_method("spawn", force=True)
    args = argparse.Namespace(confidence_threshold=0.5, config_file=CONFIGS_DIR, opts=['MODEL.WEIGHTS', MODEL_DIR], video_input=None, webcam=False)
    setup_logger(name="fvcore")
    logger = setup_logger()
    logger.info("Arguments: " + str(args))
    cfg = setup_cfg(args)
    demo = VisualizationDemo(cfg)
    return demo, logger


def panoptic_segmentation(demo, logger, args_input, args_output):
    # Perform Image Segmentation on the RGB Image
    if args_input:
        if len(args_input) == 1:
            args_input = glob.glob(os.path.expanduser(args_input[0]))
            assert args_input, "The input path(s) was not found"
        for path in tqdm.tqdm(args_input, disable=not args_output):
            # use PIL, to be consistent with evaluation
            img = read_image(path, format="BGR")
            start_time = time.time()
            predictions, visualized_output, panoptic_seg, segments_info = demo.run_on_image(img)
            logger.info(
                "{}: {} in {:.2f}s".format(
                    path,
                    "detected {} instances".format(len(predictions["instances"]))
                    if "instances" in predictions
                    else "finished",
                    time.time() - start_time,
                )
            )
            
            # Saving output to Image, comment out for actual run on the agent
            if args_output:
                if os.path.isdir(args_output):
                    assert os.path.isdir(args_output), args_output
                    out_filename = os.path.join(args_output, os.path.basename(path))
                else:
                    assert len(args_input) == 1, "Please specify a directory with args.output"
                    out_filename = args_output
                visualized_output.save(out_filename)
            else:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.imshow(WINDOW_NAME, visualized_output.get_image()[:, :, ::-1])
                if cv2.waitKey(0) == 27:
                    break  # esc to quit
        
        return panoptic_seg, segments_info
