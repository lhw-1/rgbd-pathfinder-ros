# Referenced from: https://github.com/sanchithaseshadri/PathPlanningAstar/blob/master/astar.py

import math
import numpy as np
import cv2

from astar import a_star
from map import Map
from node import Node
from transformations import cartesian_to_pixel

class PathPlanner:
    def __init__(self, img, start, goal):
        print("Building map...")
        self.map = Map(img)
        self.grid_map = self.map.grid_map
        self.img_height = len(img)
        self.img_width = len(img[0])
        self.map_height = self.map.fullheight
        self.map_width = self.map.fullwidth
        self.start = Node(start[0], start[1] + (self.img_height // 2))
        self.goal = Node(goal[0], goal[1] + (self.img_height // 2))
        self.theta = 0
        print("Map built. Planner initialized.")

    def plan(self):
        final = a_star(self.start, self.goal, self.grid_map)
        planned_paths = np.full((self.map_height, self.map_width), 0)
        if final == None:
            print("Path not found.")
            return None
        else:
            print("Constructing path..")
            path = self.construct_path(final)	# path in world coordinates
            print("path: ")
            points = []
            for step in path:
                points.append((step.x, step.y))
            # publish this path - safegoto for each of the path components
            points.reverse()
            points = points[1:]
            points.append((self.goal.x, self.goal.y))
            for p in range(len(points)):
                # print("x:", points[p][0], " y:", points[p][1])
                pixel_p = cartesian_to_pixel((points[p][0], points[p][1]), (self.map_width, self.map_height))
                print("Cx:", pixel_p[0], " Cy:", pixel_p[1])
                planned_paths[pixel_p[1]][pixel_p[0]] = 128
            # first process the points
            translate_x = points[0][0]
            translate_y = points[0][1]
            for p in range(len(points)):
                new_x = points[p][0] - translate_x
                new_y = points[p][1] - translate_y
                if self.theta == math.pi/2:
                    points[p] = [-new_y, new_x]
                elif self.theta == math.pi:
                    points[p] = [-new_x, -new_y]
                elif self.theta == -math.pi/2:
                    points[p] = [new_y, -new_x]
                else:			
                    points[p] = [new_x, new_y]
            # translate coordinates for theta
            return planned_paths

    def construct_path(self, end):
        """
        backtrack from end to construct path
        """
        current = end
        path = []	# path needs to be in world coordinates
        while current != None:
            path.append(current)
            current = current.parent
        return path
