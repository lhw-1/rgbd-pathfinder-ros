MAP_WIDTH = 400
MAP_HEIGHT = 400
LASER_MAX = 8.0

def world_to_pixel(world_points, image_size):
    world_x, world_y = world_points
    img_h, img_w = image_size
    pixel_points = [0, 0]
    pixel_points[0] = int(max((world_x / MAP_WIDTH) * img_w, 0))
    if pixel_points[0] > img_w - 1:
        pixel_points[0] = img_w - 1
    pixel_points[1] = int(max((world_y / MAP_HEIGHT) * img_h, 0))
    if pixel_points[1] > img_h - 1:
        pixel_points[1] = img_h
    pixel_points[1] = pixel_points[1]
    pixel_points[0] = img_w/2 + pixel_points[0]
    pixel_points[1] = img_h/2 - pixel_points[1]
    return pixel_points

if __name__ == '__main__':
    goal = world_to_pixel((-1, -1), (400, 400))
    print(goal)
