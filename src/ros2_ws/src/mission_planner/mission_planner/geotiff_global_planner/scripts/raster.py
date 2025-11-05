from geo_transform_utils import GeoImageTransformer
from image_planner import GridPlanner, draw_path_on_img
import os

script_dir = os.path.dirname(os.path.abspath(__file__))


# 1. Initialize the planner with your image and target grid size (to reduce A* path computation time (50x50 is found the maximum pair))
IMAGE_FILENAME = 'geo_wildcat_mosaic_mask_with_band.tif'
IMAGE_FILE = os.path.join(script_dir, 'imgs', IMAGE_FILENAME)
planner = GridPlanner(IMAGE_FILE, target_width=50, target_height=50)

result = planner.plan()

# 4. Get the path
path_grid = result['path']          # Path in grid coordinates
start_point = result['start']       # Start point used
goal_point = result['goal']         # Goal point used

# 5. Scale path to original image size
a_start_path_pixels = planner.scale_path_to_original(path_grid)

# Downsample path every 3rd element for simplicity
a_start_path_pixels = a_start_path_pixels[::3]



# 6. Drawing the generated path on top of original image for visualization purposes
OUTPUT_IMAGE_FILE = os.path.join(script_dir, 'out_imgs', f'{IMAGE_FILENAME}_with_traj.png')
draw_path_on_img(planner.img_original, a_start_path_pixels, OUTPUT_PATH_FILE=OUTPUT_IMAGE_FILE, save_to_image=True)


# 7. Get path from pixel,pixel to valid lat, lon pairs using GeoTIFF img information.

# Handler class for Pixel/GPS transformations
transformer = GeoImageTransformer(
    ref_pixel=(1976/2, 25),
    ref_latlon=(40.443026, -86.763256),
    image_size_px=(1976, 5375),
    scale_xy = (16.0 / 1280.0, 9.0  / 720.0),   # meters per pixel
    rotation_deg=-140.0,
    y_axis_down=True
)

# Convert pixels → GPS
gps_path = transformer.pixels_to_latlon(a_start_path_pixels)
for (lat, lon) in gps_path:
    #print(f'{lat}, {lon}')
    pass

# Convert GPS → nearest pixel

# sanity check
# Get the start point
# add half the distance of the image in pixels (so we know the new start point is out of the image for sure)
# this should still get the pixel pair that is closest to the given lat, long pair
for e, idx in enumerate(a_start_path_pixels):
    x, y = a_start_path_pixels[-1]
    #print(f'original {x}, {y}')
    test_px = (1976 + 1, 5375 + 1)
    print(f'out of dist {test_px}')
    latlon = transformer.pixels_to_latlon([test_px])[0]
    px_back = transformer.latlon_to_pixel(latlon)
    print(px_back)
    break

#print(f"Original pixel: {test_px}, Back-converted pixel: {px_back['u']}")
