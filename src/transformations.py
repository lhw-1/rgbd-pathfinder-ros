import math

def radians(degree):
    return (degree * math.pi / 180)

def cartesian_to_pixel(point, image_dim):
    # Given a point (x, y) in cartesian co-ordinates, convert to pixel locations
    pixel_point = [0, 0]
    image_width = image_dim[0]
    image_height = image_dim[1]
    cartesian_point = [point[0] // 1, point[1] // 1]
    pixel_point[0] = int((image_width // 2) + cartesian_point[0] - 1)
    pixel_point[1] = int(image_height - cartesian_point[1] - 1)
    return pixel_point

def pixel_to_cartesian(point, image_dim):
    # Given a point (x, y) in terms of pixel locations (i.e. arr[x][y] = pixel value), convert to cartesian co-ordinates
    cartesian_point = [0, 0]
    image_width = image_dim[0]
    image_height = image_dim[1]
    pixel_point = [point[0] // 1, point[1] // 1]
    cartesian_point[0] = pixel_point[0] + 1 - (image_width // 2)
    cartesian_point[1] = pixel_point[1] + 1 - image_height
    return cartesian_point
