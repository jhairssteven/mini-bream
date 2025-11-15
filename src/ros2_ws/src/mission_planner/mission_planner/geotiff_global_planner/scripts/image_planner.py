
import numpy as np
from PIL import Image, ImageDraw
import python_motion_planning as pmp
#import geotiff_global_planner.scripts.tif_img_editor

import heapq
import numpy as np
from python_motion_planning.utils import Env, Grid, Node
import os

class AStarWithPartial(pmp.AStar):
    """ Class override to get one valid path for the 
        closest-to-goal point when no valid path is found 
        for initial goal position. 
    """
    def extractPath(self, closed_list: dict, end_node: Node = None) -> tuple:
        """
        Extract the path based on the CLOSED list.

        Parameters:
            closed_list (dict): CLOSED list
            end_node (Node, optional): Node to reconstruct path to. Defaults to goal.

        Returns:
            cost (float): the cost of planned path
            path (list): the planning path
        """
        if end_node is None:
            end_node = closed_list[self.goal.current]  # original behavior

        cost = 0
        node = end_node
        path = [node.current]
        while node != self.start:
            node_parent = closed_list[node.parent]
            cost += self.dist(node, node_parent)
            node = node_parent
            path.append(node.current)
        path.reverse()  # optional, to have start -> end
        return cost, path
    
    def plan(self) -> tuple:
        """
        A* motion plan function with fallback to closest reachable node.

        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
            exact_path (bool): if a valid path was found to the original goal
        """
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        closest_node = self.start  # fallback node
        min_h = self.h(self.start, self.goal)

        while OPEN:
            node = heapq.heappop(OPEN)

            if node.current in CLOSED:
                continue

            # update closest node to goal
            h_val = self.h(node, self.goal)
            if h_val < min_h:
                min_h = h_val
                closest_node = node

            # goal found
            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values()), True

            for node_n in self.getNeighbor(node):
                if node_n.current in CLOSED:
                    continue
                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)
                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node

        # No path found, return path to closest node instead
        CLOSED[closest_node.current] = closest_node
        cost, path = self.extractPath(CLOSED, closest_node)
        return cost, path, list(CLOSED.values()), False


# ==============================================================================
# Planner Module
# ==============================================================================

class GridPlanner:
    """
    Handles grid creation from a binary image and path planning using A*.
    """

    def __init__(self, image_path, target_width=50, target_height=50, img_array=None):
        """
        Initializes the planner with the image and target resolution.

        Args:
            image_path (str): Path to the binary image.
            target_width (int): Width of the planning grid.
            target_height (int): Height of the planning grid.
        """
        if img_array is not None:
            # convert three channel image to a single channel binary mask, 
            # with foreground as white pixels and background as black pixels
            mask = np.any(img_array < 255, axis=-1).astype(np.uint8) * 255
            self.img_original = Image.fromarray(mask).convert('L')

        else:
            # Load the original image and convert to grayscale
            self.img_original = Image.open(image_path).convert('L')

        self.image_path = image_path
        self.target_width = target_width
        self.target_height = target_height

        # --- Resize while preserving aspect ratio ---
        original_width, original_height = self.img_original.size
        aspect_ratio = original_width / original_height

        if (target_width / target_height) > aspect_ratio:
            # Fit to height
            new_height = target_height
            new_width = int(aspect_ratio * new_height)
        else:
            # Fit to width
            new_width = target_width
            new_height = int(new_width / aspect_ratio)

        self.grid_img = self.img_original.resize((new_width, new_height), Image.Resampling.LANCZOS)

        #self.grid_img = tif_img_editor.draw_bridge_and_close_image(self.grid_img, grayscale=True)
        self.grid_img.save('./grid_img.png')
        self.grid_array = np.array(self.grid_img)
        print(self.grid_array.shape)
        self.map_height, self.map_width = self.grid_array.shape

        # Extract obstacles (black pixels)
        self.obstacles_set = self._extract_obstacles()

    def _extract_obstacles(self):
        """Return a set of (x,y) tuples representing obstacle pixels."""
        obs_indices = np.where(self.grid_array < 128)
        return set(zip(obs_indices[1], obs_indices[0]))

    def get_default_start_goal(self):
        """
        Automatically calculates start (left-down) and goal (right-top) points
        based on traversable (white) pixels.
        """
        traversable = np.where(self.grid_array > 128)
        points = list(zip(traversable[1], traversable[0]))
        if len(points) == 0:
            raise ValueError("No traversable pixels found in the image.")

        # Start: left-down (min x, max y)
        start = min(points, key=lambda p: (-p[1], p[0]))
        
        # Goal: right-top (max x, min y)
        #goal = max(points, key=lambda p: (-p[1], p[0]))
        
        # Goal: middle-top (middle x, min y)
        # Find points at the top (min y)
        points_array = np.array(points)
        min_y = np.min(points_array[:, 1])
        top_points = points_array[points_array[:, 1] == min_y]

        # Middle x of top points
        sorted_top = top_points[np.argsort(top_points[:, 0])]
        middle_index = len(sorted_top) // 2
        goal = tuple(sorted_top[middle_index])

        return start, goal
    
    def plan(self, start_point=None, goal_point=None):
        """
        Runs A* planning on the grid.

        Args:
            start_point (tuple or None): Optional start (x,y)
            goal_point (tuple or None): Optional goal (x,y)

        Returns:
            dict: {'start', 'goal', 'cost', 'path', 'expand'}
            path: the first element is the goal point, the last element is the start point
        """

        # 1. If start_point is None, calculate default start
        if start_point is None:
            start_point, _ = self.get_default_start_goal()
        
        # 2. If goal_point is None, calculate default goal
        if goal_point is None:
            _, goal_point = self.get_default_start_goal()
        
        # Create planning environment
        print('Creating grid')
        env = pmp.Grid(self.map_width, self.map_height)
        env.update(self.obstacles_set)

        if np.all(start_point == goal_point):
            raise ValueError("Start and Goal points are the same. Cannot plan.")

        # Run planner
        planner = AStarWithPartial(start=start_point, goal=goal_point, env=env)
        print('Planning')
        cost, path, expand, exact_path = planner.plan()
        
        #planner.plot.animation(path, str(planner), cost, expand)

        # Print info

        print(f"--- Generated reduced Map ---")
        print(f"Grid Size: ({self.map_width}, {self.map_height})")
        print(f"Start Position: {start_point}")
        print(f"Goal Position: {goal_point}")
        print(f"------")
        if not exact_path:
            print("No path to original goal position; returning closest partial path.")
            goal_point = path[-1]
            print(f"Closest Position to goal: {goal_point}")

        if path:
            print(f"Path cost: {cost}")
        else:
            print("No path found.")
        print(f"------")

        return {
            'start': start_point,
            'goal': goal_point,
            'cost': cost,
            'path': path,
            'expand': expand
        }

    def scale_path_to_original(self, path):
        """
        Scales grid coordinates back to original image pixel coordinates.

        Args:
            path (list of tuples): Grid path [(x,y), ...]
        """
        w, h = self.img_original.size
        uw = w / self.target_width
        uh = h / self.target_height
        # Center the path on each grid cell
        B = [(int(uw * (x + 0.5)), int(uh * (y + 0.5))) for x, y in path]
        return B

# ==============================================================================
# Visualization Module
# ==============================================================================

def draw_path_on_img(img, B, OUTPUT_PATH_FILE=None, save_to_image=False):
    """ 
    Visualize the generated path on the image, including axes, start/goal, and
    optional saving.

    Args:
        img (Image): Original PIL image
        B (list of tuples): Path coordinates scaled to original image
        OUTPUT_PATH_FILE (str): Output file path if saving
        save_to_image (bool): Whether to save the final image
    """
    img_with_path = img.copy().convert("RGB")
    draw = ImageDraw.Draw(img_with_path)

    # Draw straight lines connecting consecutive points (red)
    draw.line(B, fill=(255, 0, 0), width=3)

    # Draw yellow circles for each point
    point_radius = 2
    for p_x, p_y in B[1:-1]:
        draw.ellipse(
            (p_x - point_radius, p_y - point_radius,
             p_x + point_radius, p_y + point_radius),
            fill=(0, 0, 255)
        )

    # Draw start (green) and goal (blue)
    radius = 2
    draw.ellipse((B[0][0]-radius, B[0][1]-radius, B[0][0]+radius, B[0][1]+radius), fill=(0,255,0))
    draw.ellipse((B[-1][0]-radius, B[-1][1]-radius, B[-1][0]+radius, B[-1][1]+radius), fill=(255,0,0))

    # Draw origin and axes
    origin_x, origin_y = 0, 0
    origin_radius = 4
    axis_length = 200
    axis_width = 5
    axis_color_x = (255,0,0)
    axis_color_y = (255,255,0)
    text_color = (255,255,255)

    # X-axis
    x_axis_end = origin_x + axis_length + 5
    draw.line([(origin_x+5, origin_y), (x_axis_end, origin_y)], fill=axis_color_x, width=axis_width)
    # Y-axis
    y_axis_end = origin_y + axis_length + 5
    draw.line([(origin_x, origin_y+5), (origin_x, y_axis_end)], fill=axis_color_y, width=axis_width)
    # Draw origin (blue circle)
    draw.ellipse((origin_x-origin_radius, origin_y-origin_radius,
                  origin_x+origin_radius, origin_y+origin_radius), fill=(0,0,255))

    # Axis labels
    try:
        from PIL import ImageFont
        from matplotlib import font_manager
        font_path = font_manager.findfont('DejaVu Sans', fallback_to_default=True)
        font = ImageFont.truetype(font_path, size=60)
    except IOError:
        font = ImageFont.load_default()

    draw.text((x_axis_end + 10, origin_y + 10), "X", font=font, fill=text_color)
    draw.text((origin_x + 30, y_axis_end + 10), "Y", font=font, fill=text_color)

    # Save if requested
    if save_to_image and OUTPUT_PATH_FILE:
        try:
            img_with_path.save(OUTPUT_PATH_FILE)
            print(f"Saved final image with path to: {OUTPUT_PATH_FILE}")
        except IOError as e:
            print(f"Error saving path image: {e}")

    return img_with_path

# ==============================================================================
# Main Execution
# ==============================================================================

def python_motion_planning_lib_main():
    # Import the planner and visualize function
    #from grid_planner_module import GridPlanner, visualize

    # 1. Initialize the planner with your image and target grid size
    base_path = '/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/a_start_generator/scripts/imgs'
    IMAGE_FILE = base_path + 'geo_wildcat_mosaic_mask.tif'
    planner = GridPlanner(IMAGE_FILE, target_width=50, target_height=50)

    # 2. Optional: Provide a start_point from another module
    # For example, you might calculate start_point externally
    # start_point = (10, 40)
    # goal_point = (45, 5)

    # 3. Run the planner
    # Option 1: Use automatic start/goal
    result = planner.plan()

    # Option 2: Provide start/goal explicitly
    #result = planner.plan(start_point=start_point)

    # 4. Get the path
    path_grid = result['path']          # Path in grid coordinates
    start_point = result['start']       # Start point used
    goal_point = result['goal']         # Goal point used

    # 5. Scale path to original image size (for visualization or other module input)
    a_start_path = planner.scale_path_to_original(path_grid)


    # 6. Optional: Visualize the path
    output_image_file = f'{IMAGE_FILE}_with_traj.png'
    visualize(planner.img_original, a_start_path, OUTPUT_PATH_FILE=output_image_file, save_to_image=True)

    print(f"Start: {start_point}, Goal: {goal_point}")

import imageio
import time
import pyastar2d

class AStartPlanner():
    def __init__(self, save_output=True, output_dir='./', filename='img_with_path'):
        self.save_output = save_output
        self.output_dir = output_dir

    def plan(self, image_path=None, image_array=None, start=None, goal=None, save_output=True, output_dir=None, filename='img_with_path'):
        """ 
        image_path: A .tiff or .png or .jpg image. Traversable pixels are white, non traversable pixels are black
        start: Pixel to start path. Pixel coordinates (x, y) of a valid traversable pixel
        goal: Pixel to finish path. Pixel coordinates (x, y) of a valid traversable pixel
        Returns:
            path (ndarray) Format: [[i0, j0], [i1, j1], ...]: The pixel coordinates of the path in 'image_path'.
        """

        grid, maze = self.read_img_as_grid(image_path=image_path, image_array=image_array)
        start, goal = self.get_start_and_goal(grid, start, goal)

        print(f"[A* Planner] Using grid with shape {grid.shape}. Start: {start}, Goal: {goal}")
        t0 = time.time()
        path = pyastar2d.astar_path(grid, start, goal, allow_diagonal=False)
        dur = time.time() - t0
        print(f"[A* Planner] Found path of length {path.shape[0]} elements in {dur:.6f}s")

        if path.shape[0] > 0:
            if save_output:
                self.save_path_to_img(path, maze, output_dir, filename=filename)
        else:
            print("[A* Planner] No path found")

        return path

    def save_path_to_img(self, path, maze, output_dir, filename='img_with_path'):
        
        #maze = maze.astype(np.int8) * 255
        maze = np.stack((maze.astype(np.uint8),) * 3, axis=-1) # convert to 3 channel
        # Update path pixels to red color
        maze[path[:, 0], path[:, 1]] = (255, 0, 0)

        os.makedirs(output_dir, exist_ok=True)
        save_path = os.path.join(output_dir, filename + '.png')
        
        print(f"[A* Planner] Plotting path to {save_path}")
        imageio.imwrite(save_path, maze)
    
    def get_start_and_goal(self, grid, start=None, goal=None):
        """
        Get start and goal positions in the grid.
        If provided start/goal are invalid (out of bounds or non-traversable),
        find the nearest valid cell (value == 1) using Euclidean distance.
        """

        # Get all valid traversable points
        valid_points = np.argwhere(grid == 1)

        def find_nearest_valid(point):
            """Return the nearest valid cell to 'point' using Euclidean distance."""
            if valid_points.size == 0:
                raise ValueError("Grid has no traversable (value=1) cells.")
            distances = np.linalg.norm(valid_points - np.array(point), axis=1)
            nearest_idx = np.argmin(distances)
            return valid_points[nearest_idx]

        def is_valid(point):
            """Check if a point is inside grid and traversable."""
            r, c = point
            if 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]:
                return grid[r, c] == 1
            return False

        if start is not None and goal is not None:
            # Ensure start and goal are numpy arrays
            start, goal = np.array(start), np.array(goal)

            # Ensure are traversable pixels or find closest valid if necessary
            if not is_valid(start):
                print(f'[A* Planner] Given start point: {start} is not traversable. Finding closest...')
                start = find_nearest_valid(start)

            if not is_valid(goal):
                print(f'[A* Planner] Given goal point: {goal} is not traversable. Finding closest...')
                goal = find_nearest_valid(goal)
        else:
            # if not start, goal is provided, find some automatically
            if start is None:
                # start is the first index in the bottom-most row that has a 1
                rows_with_ones = np.where(np.any(grid == 1, axis=1))[0]
                last_row_idx = rows_with_ones[-1]
                start_j, = np.where(grid[last_row_idx, :] == 1)
                start = np.array([last_row_idx, start_j[0]])
            if goal is None:
                # end is the last index in the top-most row that has a 1
                rows_with_ones = np.where(np.any(grid == 1, axis=1))[0]
                first_row_idx = rows_with_ones[0]  # last row that has a 1
                end_i, = np.where(grid[first_row_idx, :] == 1)
                goal = np.array([first_row_idx, end_i[-1]])
        
        return start, goal
    
    def read_img_as_grid(self, image_path, image_array=None):
        """
        Read an image and convert it into a cost grid for navigation.

        Parameters
        ----------
        image_path : str
            Path to the image file on disk. Ignored if `image_array` is provided.
        image_array : np.ndarray or None, optional
            A pre-loaded image array. If None, the image is read from `image_path`.

        Returns
        -------
        grid : np.ndarray (dtype=float32, shape=(H, W))
            A 2D float32 cost grid where:
                - grid[y, x] = 1.0   → traversable (white pixel)
                - grid[y, x] = inf   → non-traversable (black pixel)

        image_array : np.ndarray (dtype=uint8, shape=(H, W))
            The grayscale 2D uint8 image (0-255), after conversion and binarization.
        """

        if image_array is None:
            image_array = imageio.imread(image_path)

            if image_array is None:
                print(f"No file found: {image_path}")
                return
            else:
                print(f"Loaded Image of shape {image_array.shape}")

        if image_array.ndim == 3:
            print("Input image has 3 channels; converting to grayscale.")
            image_array = np.mean(image_array, axis=2).astype(np.uint8)
        
        # Get a binary mask with white color for traversable area
        if np.max(image_array) == 1:
            image_array[np.where(image_array == 1)] = 255
            image_array[np.where(image_array == 0)] = 0
        elif np.max(image_array) == 255:
            image_array[np.where(image_array < 128)] = 0
            image_array[np.where(image_array > 128)] = 255


        # Create a cost grid based on the grayscale-color image
        grid = image_array.astype(np.float32)
        grid[grid == 0] = np.inf # Black pixels asign infinite cost
        grid[grid == 255] = 1 # White pixels cost of 1

        assert grid.min() == 1, "cost of moving must be at least 1"

        return grid, image_array



import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        "An example of using pyastar2d to find the solution to a maze"
    )
    parser.add_argument(
        "--input", type=str, default="/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/geotiff_global_planner/assets/global_map/river_map.png",
        help="Path to the black-and-white image to be used as input.",
    )
    parser.add_argument(
        "--output", type=str, default="/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/geotiff_global_planner/assets/output/river_map.png", 
        help="Path to where the output will be written",
    )

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    #python_motion_planning_lib_main()
    args = parse_args()

    data = np.load('/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/depth_bev_data.npz')
    BEV_start=data['BEV_start']
    BEV_goal=data['BEV_goal']
    bev_image_vis=data['bev_image_vis']
    
    print('BEV_start', BEV_start)
    print('BEV_goal', BEV_goal[1], BEV_goal[0])
    print("bev_image_vis.shape", bev_image_vis.shape)
    
    maze_inv = np.copy(bev_image_vis)
    maze_inv[np.where(bev_image_vis < 250)] = 255
    maze_inv[np.where(bev_image_vis > 250)] = 0

    astart_planner = AStartPlanner(save_output=True, output_dir=args.output)
    path = astart_planner.plan(image_path=args.input, 
                               image_array=maze_inv, 
                               start=np.array([BEV_start[1], BEV_start[0]]),
                               goal=np.array([BEV_goal[1], BEV_goal[0]]),
                               save_output=True, 
                               output_dir=args.output)
