# Referenced from: https://github.com/sanchithaseshadri/PathPlanningAstar/blob/master/buildMap.py

"""
Given an image of a map, create an occupancy grid map from it.
This occupancy grid will be used in the A-star search.
"""
import math
import numpy as np
import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from transformations import cartesian_to_pixel, pixel_to_cartesian


class Map:
	"""
	The Map class builds a map from a given image (assumed to be a binary image of Black and White).
	For each pixel on the map, store the value of the pixel:
	True if the pixel is traversable and obstacle-free, False otherwise.
	"""
	def __init__(self, np_img):
		"""
		Construct an occupancy grid map from the image.
		"""
		self.pixels = np_img
		self.height = len(self.image)
        self.width = len(self.image[0])
		self.grid_map = []
		for x in range(self.width):
			row = []
			for y in range(self.height):
				if self.pixels[x][y][0] == 0:
					row.append(False)
				else:
					row.append(True)
			self.grid_map.append(row)
