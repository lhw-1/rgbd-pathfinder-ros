import numpy as np

from PIL import Image as PILImage
from PIL import ImageDraw

# Parameters
TRAVERSABLE = [3,11,52,53,59,91,96,121]


def generate_mapping(segments_info):
    mapping = {0:-1}
    for obj in segments_info:
        mapping[obj["id"]] = obj["category_id"]
    return mapping


def map_segmentation_ids(panoptic_seg, segments_info):
    # Map the ids to the correct category ids using segments_info
    segments_mapping = generate_mapping(segments_info)
    apply_mapping = lambda id, segments_info : segments_info[id]
    apply_mapping = np.vectorize(apply_mapping)
    traversable_seg = apply_mapping(panoptic_seg, segments_mapping)

    for row in traversable_seg:
        # Filter all non-traversable areas to -1
        for i in range(len(row)):
            if row[i] not in TRAVERSABLE:
                row[i] = -1

    return traversable_seg


def calculate_traversable_paths(panoptic_seg):

    # TODO: Make this more efficient using np.vectorize or otherwise
    traversable_areas = []
    traversable_paths = []
    row_idx = 0
    for row in panoptic_seg:
        # Filter all non-traversable areas to -1 and find all traversable areas & traversable paths
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


def draw_paths(traversable_seg_bev, traversable_paths):

    # Open Image
    IM_WIDTH = len(rgb_img[0])
    IM_HEIGHT = len(rgb_img)
    rgb_img = PILImage.fromarray(traversable_seg_bev.astype('uint8'), 'RGB')
    rgb_img_map = rgb_img.load()
    rgb_img_traversable = PILImage.new(rgb_img.mode, rgb_img.size)
    rgb_img_traversable_map = rgb_img_traversable.load()
    for i in range(rgb_img.size[0]):
        for j in range(rgb_img.size[1]):
            rgb_img_traversable_map[i,j] = rgb_img_map[i,j]
    draw = ImageDraw.Draw(rgb_img_traversable)
    traversable_x = IM_WIDTH // 2
    traversable_y = IM_HEIGHT // 2
    starting_x = IM_WIDTH // 2
    starting_y = IM_HEIGHT - 1

    # Choose the next traversable node
    traversable_node_found = False
    starting_x_candidates = []
    for path in traversable_paths:
        draw.ellipse((path['x'] - 2, path['y'] - 2, path['x'] + 2, path['y'] + 2), fill=(255, 0, 0))
        if path['y'] > (11 * IM_HEIGHT / 16) and path['y'] < (12 * IM_HEIGHT / 16):
            traversable_node_found = True
            traversable_x = path['x']
            traversable_y = path['y']
        if path['y'] == IM_HEIGHT - 1:
            starting_x_candidates.append(path['x'])

    # Draw next traversable node more clearly
    draw.ellipse((path['x'] - 3, path['y'] - 3, path['x'] + 3, path['y'] + 3), fill=(0, 0, 0))

    # Get the starting point closest to the next traversable node    
    starting_x = min(starting_x_candidates, key = lambda x: abs(x - traversable_x))

    # Return values as needed
    traversable_array = np.array(rgb_img_traversable)[:, :, 2::-1]    
    return [traversable_x, traversable_y], [starting_x, starting_y], traversable_node_found, traversable_array
