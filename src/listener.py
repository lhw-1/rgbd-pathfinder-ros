#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import mask2former
TRAVERSABLE = [3,11,52,53,59,91,96,121]
MODEL_DIR = "../data/model_final_5c90d4.pkl"
CONFIGS_DIR = "../data/maskformer2_R50_bs16_160k.yaml"
IMAGE_FILENAME = "../data/test.png"

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
    STEER_THRESHOLD = IM_WIDTH // 8

    # Perform Mask2Former Segmentation
    panoptic_seg, segments_info = mask2former.run_segmentation(MODEL_DIR, CONFIGS_DIR, [IMAGE_FILENAME])

    # Calculate traversable paths and areas
    traversable_areas, traversable_paths = calculate_traversable_paths(panoptic_seg, segments_info)

    # Obtain a sample path
    traversable_x = IM_WIDTH // 2
    for path in traversable_paths:
        if path['y'] > (5 * rgb_img.size[1] / 8) and path['y'] < (7 * rgb_img.size[1] / 8):
            traversable_x = path['x']

    dx = (traversable_x - (IM_WIDTH // 2))

    # Change steer based on dx
    if dx > STEER_THRESHOLD:
        steer = 0.5
    elif dx < -STEER_THRESHOLD:
        steer = -0.5
    else:
        steer = 0.0
    publish_steer(steer)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener()
