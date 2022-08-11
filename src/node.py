# Referenced from: https://github.com/sanchithaseshadri/PathPlanningAstar/blob/master/node.py

from transformations import cartesian_to_pixel, pixel_to_cartesian

SAFETY_OFFSET = 5
SIMILARITY_THRESHOLD = 0.1

class Node:
	def __init__(self, x, y, theta=0.0, parent=None):
		self.x = x
		self.y = y
		self.theta = theta
		self.parent = parent
		# f(n) = h(n) + g(n)
		self.f = 0
		self.h = 0
		self.g = 0

	def euclidean_distance(self, goal):
		"""
		Method to compute distance from current position to the goal
		@arg	goal 	Node object with x, y, theta
		@returns 	euclidean distance from current point to goal
		"""
		return math.sqrt(math.pow((goal.x-self.x),2) + math.pow((goal.y-self.y),2))

	def apply_move(self, move):
		"""
		Apply the given move to current position
		@arg 	move 	[length, dtheta]
		"""
		theta_new = self.theta + move[1]
		x_new = self.x + math.cos(theta_new) * move[0]	# d.cos(theta)
		y_new = self.y + math.sin(theta_new) * move[0]  # d.sin(theta)
		return Node(x_new, y_new, theta_new)

	def is_move_valid(self, grid_map, move):
		"""
		Return true if required move is legal
		"""
		goal = self.apply_move(move)
		# convert goal coordinates to pixel coordinates before checking this
		goal_pixel = cartesian_to_pixel((goal.x, goal.y), (len(grid_map[0]), len(grid_map)))
		# check if too close to the walls
		if goal_pixel[0] >= SAFETY_OFFSET and not grid_map[goal_pixel[0]-SAFETY_OFFSET][goal_pixel[1]]:
			return False
		if goal_pixel[1] >= SAFETY_OFFSET and not grid_map[goal_pixel[0]][goal_pixel[1]-SAFETY_OFFSET]:
			return False
		if goal_pixel[0] >= SAFETY_OFFSET and goal_pixel[1] >= SAFETY_OFFSET and not grid_map[goal_pixel[0]-SAFETY_OFFSET][goal_pixel[1]-SAFETY_OFFSET]:
			return False
		if grid_map[goal_pixel[0]][goal_pixel[1]]:
			return True
		return False

	def is_valid(self, grid_map):
		"""
		Return true if the location on the map is valid, ie, in obstacle free zone
		"""
		goal_pixel = cartesian_to_pixel((self.x, self.y), (len(grid_map[0]), len(grid_map)))
		if grid_map[goal_pixel[0]][goal_pixel[1]]:
			return True
		return False

	def is_similar(self, other):
		"""
		Return true if other node is in similar position as current node
		"""
		return self.euclidean_distance(other) <= SIMILARITY_THRESHOLD
