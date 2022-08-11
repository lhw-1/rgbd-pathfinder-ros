#!/usr/bin/env pythonget_cfgCvBridgeError

import cv2
import numpy as np
import rospy
import ros_numpy
import sys

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# Local Imports
from converter import convert_to_bev, convert_to_monochrome
from node import Node
from pathfinder import calculate_traversable_paths, draw_paths, map_segmentation_ids
from pathplanner import PathPlanner
from segmentation import load_model, panoptic_segmentation

# Paths
IMAGE_DIR = "../data/" # Input from camera
RGBDP_DIR = "../data/rgbdp_outputs/"

# Parameters
IM_WIDTH = 640
IM_HEIGHT = 480
STEER_THRESHOLD = IM_WIDTH // 32        # Determine whether a diagonal line is considered forward or rotate
CENTRAL_THRESHOLD = IM_WIDTH // 32      # Determine when to move the robot to the center
DEMO, LOGGER = load_model()
LINEAR_X_VEL = 0.4                # Linear x Velocity
LINEAR_Y_VEL = 0.4                # Linear y Velocity
ANGULAR_Z_VEL = 0.5               # Angular z Velocity
LINEAR_X_COUNTER = 50                   # Determine duration for the robot to move forward
LINEAR_X_BACKWARD_COUNTER = 30          # Determine duration for the robot to move backward
LINEAR_Y_COUNTER = 30                   # Determine duration for the robot to move sideways
ANGULAR_Z_COUNTER = 10                  # Determine duration for the robot to rotate

# Goals
# These variables are set according to the map grid system, not the actual pixel locations
START_POINT = (0, 0)                        # Start is Bottom Center of the Image (Current Position)
GOAL_POINT = (0, (IM_HEIGHT // 4) * 5)      # Goal is North of Current Position, beyond the Image
# GOAL_POINT = [sys.argv[1], sys.argv[2]]

# Global Variables (These are modified as the script runs)
IMAGE_COUNTER = 0
FIRST_IMAGE_RECEIVED = False
LINEAR_X = 0.0
LINEAR_Y = 0.0
ANGULAR_Z = 0.0



def publish_steer(linear_x, linear_y, angular_z):
    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    vel_msg.linear.y = linear_y
    vel_msg.angular.z = angular_z
    vel_pub.publish(vel_msg)


def callback(data):
    
    # Access global variables
    global IMAGE_COUNTER
    global FIRST_IMAGE_RECEIVED
    global LINEAR_X
    global LINEAR_Y
    global ANGULAR_Z

    # Receive Image from camera
    print("Processing frame | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec())
    delayed_secs = (rospy.Time.now() - data.header.stamp).to_sec()
    if delayed_secs > 0.1:
        return
    br = CvBridge()
    rospy.loginfo('Image received...')
    image = ros_numpy.numpify(data)
    image = image[:, :, ::-1]
    image = cv2.resize(image, (IM_WIDTH, IM_HEIGHT), interpolation = cv2.INTER_AREA)
    # cv2.imshow("img", image)
    # cv2.waitKey(10)
    # rospy.sleep(1)
    cv2.imwrite(IMAGE_DIR + str(IMAGE_COUNTER) + ".png", image)
    
    # Perform Mask2Former Segmentation
    panoptic_seg, segments_info = panoptic_segmentation(DEMO, LOGGER, [IMAGE_DIR + str(IMAGE_COUNTER) + ".png"], '../data/outputs/m2f_' + str(IMAGE_COUNTER) + '.png')

    # Convert the Panoptic Segmentation Tensor into NumPy array
    panoptic_seg = panoptic_seg.cpu().clone().detach().numpy()

    # Map the Segmentation image to traversable / non-traversable categories
    traversable_seg = map_segmentation_ids(panoptic_seg, segments_info)

    # Convert Segmentation to Monochrome
    traversable_seg_mnc = convert_to_monochrome(traversable_seg)

    # Convert Monochrome to Bird's Eye View
    traversable_seg_bev = convert_to_bev(traversable_seg_mnc)

    # TODO TODO TODO Using the BEV Calculated, map it into the ROS Global Planner & obtain the traversable path. From there determine direction to travel
    planner = PathPlanner(traversable_seg_bev, START_POINT, GOAL_POINT)
	planner.plan()









    # Calculate traversable paths and areas
    _, traversable_paths = calculate_traversable_paths(traversable_seg_bev)

    # Calculate and draw central path
    traversable_point, starting_point, traversable_node_found, traversable_array = draw_paths(traversable_seg_bev, traversable_paths)

    # Once the path for the first image is found, allow SPOT to move
    FIRST_IMAGE_RECEIVED = True

    # Display and save the image with traversable path
    cv2.imwrite(RGBDP_DIR + "rgbdp_" + str(IMAGE_COUNTER) + ".png", traversable_array)
    cv2.imshow("seg", traversable_array)
    cv2.waitKey(10)

    # Increment Image counter
    IMAGE_COUNTER += 1

    # Publish command to change steer based on diff_x
    diff_x = traversable_point[0] - starting_point[0]
    diff_start = starting_point[0] - (IM_WIDTH // 2)
    movement_counter = 0

    if traversable_node_found:
        print("Next Node found at X: " + str(traversable_point[0]) + ", Y: " + str(traversable_point[1]) + ".")
    else:
        print("No Suitable Node found.")

    if not traversable_node_found:
        LINEAR_X = -LINEAR_X_VEL
        LINEAR_Y = 0.0
        ANGULAR_Z = 0.0
        print("Action taken: Move Backward.")
        movement_counter = LINEAR_X_BACKWARD_COUNTER
    elif diff_start > CENTRAL_THRESHOLD:
        LINEAR_X = 0.0
        LINEAR_Y = -LINEAR_Y_VEL
        ANGULAR_Z = 0.0
        print("Action taken: Move Right.")
        movement_counter = LINEAR_Y_COUNTER
    elif diff_start < -CENTRAL_THRESHOLD:
        LINEAR_X = 0.0
        LINEAR_Y = LINEAR_Y_VEL
        ANGULAR_Z = 0.0
        print("Action taken: Move Left.")
        movement_counter = LINEAR_Y_COUNTER
    elif diff_x > STEER_THRESHOLD:
        LINEAR_X = 0.0
        LINEAR_Y = 0.0
        ANGULAR_Z = -ANGULAR_Z_VEL
        print("Action taken: Rotate Right.")
        movement_counter = ANGULAR_Z_COUNTER
    elif diff_x < -STEER_THRESHOLD:
        LINEAR_X = 0.0
        LINEAR_Y = 0.0
        ANGULAR_Z = ANGULAR_Z_VEL
        print("Action taken: Rotate Left.")
        movement_counter = ANGULAR_Z_COUNTER
    else:
        LINEAR_X = LINEAR_X_VEL
        LINEAR_Y = 0.0
        ANGULAR_Z = 0.0
        print("Action taken: Move Forward.")
        movement_counter = LINEAR_X_COUNTER
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and movement_counter > 0:
        publish_steer(LINEAR_X, LINEAR_Y, ANGULAR_Z)
        rate.sleep()
        movement_counter -= 1

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=1, buff_size = 1000000000)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #if not FIRST_IMAGE_RECEIVED:
        #    publish_steer(0.0, 0.0)
        #elif ANGULARZ == 0.0:
        #    #publish_steer(0.4, ANGULARZ)
        #    publish_steer(0.3, ANGULARZ)
        #else:
        #    publish_steer(0.0, ANGULARZ)
        rate.sleep()

if __name__ == '__main__':
    vel_pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=10)
    listener()
