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
from PIL import Image as PILImage
from PIL import ImageDraw

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
IMAGE_PATH = "../data/" # Input from camera
IMAGE_COUNTER = 0
RGBDP_PATH = "../data/rgbdp_outputs/"
ANGULAR_Z = 0
DEMO = 0
LOGGER = 0

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


def load_model():
    # Load Model for the Segmentation    
    mp.set_start_method("spawn", force=True)
    global IMAGE_COUNTER
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


def publish_steer(linear_x, angular_z):
    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z
    vel_pub.publish(vel_msg)
   # print(vel_msg)


def callback(data):
    print("Processing frame | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec())
    delayed_secs = (rospy.Time.now() - data.header.stamp).to_sec()
    if delayed_secs > 0.1:
        return
    br = CvBridge()
    rospy.loginfo('Image received...')
    image = ros_numpy.numpify(data)
    image = image[:, :, ::-1]
    image = cv2.resize(image, (640, 480), interpolation = cv2.INTER_AREA)
    #cv2.imshow("img", image)
    #cv2.waitKey(10)
    #rospy.sleep(1)
    #return
    global IMAGE_COUNTER

    cv2.imwrite(IMAGE_PATH + str(IMAGE_COUNTER) + ".png", image)
    IM_WIDTH = 640
    IM_HEIGHT = 480
    STEER_THRESHOLD = IM_WIDTH // 20

    # Perform Mask2Former Segmentation
    global DEMO
    global LOGGER
    panoptic_seg, segments_info = panoptic_segmentation(DEMO, LOGGER, [IMAGE_PATH + str(IMAGE_COUNTER) + ".png"], '../data/outputs/m2f_' + str(IMAGE_COUNTER) + '.png')

    # Calculate traversable paths and areas
    _, traversable_paths = calculate_traversable_paths(panoptic_seg, segments_info)
    
    rgb_img = PILImage.open("../data/outputs/m2f_" + str(IMAGE_COUNTER) + ".png")
    rgb_img_map = rgb_img.load()
    rgb_img_traversable = PILImage.new(rgb_img.mode, rgb_img.size)
    rgb_img_traversable_map = rgb_img_traversable.load()
    for i in range(rgb_img.size[0]):
        for j in range(rgb_img.size[1]):
            rgb_img_traversable_map[i,j] = rgb_img_map[i,j]

    # Obtain a sample path
    draw = ImageDraw.Draw(rgb_img_traversable)
    traversable_x = IM_WIDTH // 2
    traversable_y = IM_HEIGHT // 2
    for path in traversable_paths:
        draw.ellipse((path['x'] - 2, path['y'] - 2, path['x'] + 2, path['y'] + 2), fill=(255, 0, 0))
        # Choose a non-deterministic point between 7/10 and 8/10 of the image height
        # There is bias towards paths to the right
        if path['y'] > (7 * IM_HEIGHT // 10) and path['y'] < (8 * IM_HEIGHT // 10):
            traversable_x = path['x']
            traversable_y = path['y']
    
    dx = (traversable_x - (IM_WIDTH // 2))
    angular_z = 0.0
    # Change angular_z based on dx
    print("Path found at X: " + str(traversable_x) + ", Y: " + str(traversable_y) + ".")
    dx = (traversable_x - (IM_WIDTH // 2))
    if dx > STEER_THRESHOLD:
        angular_z = -0.5
        print("Action to be taken: Rotate Right.")
    elif dx < -STEER_THRESHOLD:
        angular_z = 0.5
        print("Action to be taken: Rotate Left.")
    else:
        angular_z = 0.0
        print("Action to be taken: Move Forward.")
    
    #rgb_img_traversable.save(RGBDP_PATH + "traversable_" + str(IMAGE_COUNTER) + ".png", "PNG")
    img_cv2 = np.array(rgb_img_traversable)[:, :, 2::-1]
    cv2.imshow("seg", img_cv2)
    cv2.waitKey(10)

    #rgb_img_traversable.show()
    global ANGULAR_Z
    ANGULAR_Z = angular_z
    IMAGE_COUNTER += 1

def listener():
    rospy.init_node('listener', anonymous=True)
    global DEMO
    global LOGGER
    DEMO, LOGGER = load_model()
    rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size = 1, buff_size = 2147483647) # No reason for buff size other than that it's a big number
    #rospy.spin()
    rate = rospy.Rate(10)
    global ANGULAR_Z
    while not rospy.is_shutdown():
        if ANGULAR_Z == 0.0:
            publish_steer(0.4, ANGULAR_Z)
        else:
            publish_steer(0.0, ANGULAR_Z)
        rate.sleep()

if __name__ == '__main__':
    #vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=10)
    listener()
