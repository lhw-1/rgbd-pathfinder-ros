#!/usr/bin/env python

import argparse
import cv2
import glob
import multiprocessing as mp
import numpy as np
import os
import rospy
import ros_numpy
import time
import tqdm
import warnings

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.projects.deeplab import add_deeplab_config
from detectron2.utils.logger import setup_logger

from Mask2Former.mask2former import add_maskformer2_config
from predictor import VisualizationDemo

WINDOW_NAME = "mask2former demo"
TRAVERSABLE = [3,11,52,53,59,91,96,121]
MODEL_DIR = "../data/model_final_5c90d4.pkl"
CONFIGS_DIR = "../data/maskformer2_R50_bs16_160k.yaml"
IMAGE_FILENAME = "../data/test.png"

def generate_mapping(segments_info):
    mapping = {0:-1}
    for obj in segments_info:
        mapping[obj["id"]] = obj["category_id"]
    return mapping

def calculate_traversable_paths(panoptic_seg, segments_info):

    # Convert the Panoptic Segmentation Tensor into NumPy array
    panoptic_seg_arr = panoptic_seg.cpu().clone().detach().numpy()

    # Map the ids to the correct category ids using segments_info
    segments_mapping = generate_mapping(segments_info)
    apply_mapping = lambda id, segments_info : segments_info[id]
    apply_mapping = np.vectorize(apply_mapping)
    panoptic_seg_arr = apply_mapping(panoptic_seg_arr, segments_mapping)

    # TODO: Make this more efficient using np.vectorize or otherwise
    # TODO: There must be a better way to iterate over a NumPy array
    # TODO: Forking roads may require more consideration
    traversable_areas = []
    traversable_paths = []
    row_idx = 0
    for row in panoptic_seg_arr:
        # Filter all non-traversable areas to -1
        # Append all traversable areas to traversable_areas (list of dictionaries)
        # For every traversable area, find the center
        count = 0
        for i in range(len(row)):
            if row[i] not in TRAVERSABLE:
                count = 0
                row[i] = -1
            elif i == len(row) - 1 or row[i+1] not in TRAVERSABLE:
                traversable_areas.append({'row': row_idx, 'id': row[i], 'start': i - count, 'end': i})
                traversable_paths.append({'id': row[i], 'x': int((2 * i - count) / 2), 'y': row_idx})
            else:
                count = count + 1
        row_idx = row_idx + 1

    return traversable_areas, traversable_paths

def setup_cfg(args):
    # load config from file and command-line arguments
    cfg = get_cfg()
    add_deeplab_config(cfg)
    add_maskformer2_config(cfg)
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    cfg.freeze()
    return cfg

def panoptic_segmentation():
    # Perform Image Segmentation on the RGB Image    
    mp.set_start_method("spawn", force=True)
    args = argparse.Namespace(confidence_threshold=0.5, config_file=CONFIGS_DIR, input=[IMAGE_FILENAME], opts=['MODEL.WEIGHTS', MODEL_DIR], output='../data/outputs/', video_input=None, webcam=False)
    setup_logger(name="fvcore")
    logger = setup_logger()
    logger.info("Arguments: " + str(args))
    cfg = setup_cfg(args)

    demo = VisualizationDemo(cfg)

    if args.input:
        if len(args.input) == 1:
            args.input = glob.glob(os.path.expanduser(args.input[0]))
            assert args.input, "The input path(s) was not found"
        for path in tqdm.tqdm(args.input, disable=not args.output):
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
            if args.output:
                if os.path.isdir(args.output):
                    assert os.path.isdir(args.output), args.output
                    out_filename = os.path.join(args.output, os.path.basename(path))
                else:
                    assert len(args.input) == 1, "Please specify a directory with args.output"
                    out_filename = args.output
                visualized_output.save(out_filename)
            else:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.imshow(WINDOW_NAME, visualized_output.get_image()[:, :, ::-1])
                if cv2.waitKey(0) == 27:
                    break  # esc to quit
        
        return panoptic_seg, segments_info

def publish_steer(steer = 0.0):
    vel_msg = Twist()
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)
    print(vel_msg)

def callback(data):
    br = CvBridge()
    rospy.loginfo('Image received...')
    image = ros_numpy.numpify(data)
    image = image[:, :, ::-1]
    image = cv2.resize(image, (640, 480), interpolation = cv2.INTER_AREA)
    cv2.imwrite(IMAGE_FILENAME, image)
    IM_WIDTH = 640
    IM_HEIGHT = 480
    STEER_THRESHOLD = IM_WIDTH // 8

    # Perform Mask2Former Segmentation
    panoptic_seg, segments_info = panoptic_segmentation()

    # Calculate traversable paths and areas
    _, traversable_paths = calculate_traversable_paths(panoptic_seg, segments_info)

    # Obtain a sample path
    traversable_x = IM_WIDTH // 2
    traversable_y = IM_HEIGHT // 2
    for path in traversable_paths:
        if path['y'] > (5 * IM_HEIGHT / 8) and path['y'] < (7 * IM_HEIGHT / 8):
            traversable_x = path['x']
            traversable_y = path['y']

    dx = (traversable_x - (IM_WIDTH // 2))

    # Change steer based on dx
    print("Path found at X: " + str(traversable_x) + ", Y: " + str(traversable_y) + ".")
    dx = (traversable_x - (IM_WIDTH // 2))
    if dx > STEER_THRESHOLD:
        steer = 0.5
        print("Action to be taken: Rotate Right.")
    elif dx < -STEER_THRESHOLD:
        steer = -0.5
        print("Action to be taken: Rotate Left.")
    else:
        steer = 0.0
        print("Action to be taken: Move Forward.")
        
    publish_steer(steer)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener()
