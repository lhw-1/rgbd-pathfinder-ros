# Referenced from: https://github.com/sanchithaseshadri/PathPlanningAstar/blob/master/buildMap.py

"""
Given an image of a map, create an occupancy grid map from it.
This occupancy grid will be used in the A-star search.
"""

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
        self.height = len(np_img)
        self.width = len(np_img[0])
        self.fullheight = self.height // 2 * 2 + self.height
        self.fullwidth = self.width // 2 * 2 + self.width

        self.grid_map = []
        for x in range(self.fullheight):
            row = []
            if x < self.height // 2:
                for y in range(self.fullwidth):
                    row.append(True)
            elif x < ((self.height // 2) + self.height):
                for y in range(self.fullwidth):
                    if y < self.width // 2:
                        row.append(True)
                    elif y < ((self.width // 2) + self.width):
                        if self.pixels[x - self.height // 2][y - self.width // 2][0] == 0:
                            row.append(False)
                        else:
                            row.append(True)
                    else:
                        row.append(True)
            else:
                for y in range(self.fullwidth):
                    row.append(True)		
            self.grid_map.append(row)
