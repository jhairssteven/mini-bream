import numpy as np
from PIL import Image, ImageDraw
import python_motion_planning as pmp 

def create_planner_from_image(image_path, img, target_width=500, target_height=500):
    """
    Reads a binary image, forces the resolution to 500x500, saves the resulting 
    image, and then uses it to create an A* planner environment. 
    Start (left-down) and Goal (right-top) are automatically set.

    Args:
        image_path (str): The file path to the binary (black and white) image.
        target_width (int): The desired width of the final grid.
        target_height (int): The desired height of the final grid.

    Returns:
        tuple: (cost, path, expand) from the A* planning result, or (None, None, None) on failure.
    """
    
    
    # Resize the image directly to the target resolution
    img_resized = img.resize((target_width, target_height), Image.Resampling.LANCZOS)
    
    # --- SAVE THE RESIZED IMAGE ---
    #try:
    #    output_image_path = f'{target_width}x{target_height}_{image_path}'
    #    img_resized.save(output_image_path)
    #    print(f"Saved grid image to: {output_image_path}")
    #except IOError as e:
    #    print(f"Error saving image: {e}")
    #    return None, None, None
    # ------------------------------
    
    # Convert image to a NumPy array (0=Black/Obstacle, 255=White/Traversable)
    img_array = np.array(img_resized)
    
    # Determine Grid Dimensions (will be target_width x target_height)
    map_height, map_width = img_array.shape
    
    # 2. Identify Traversable Area and Obstacles
    
    # White pixels (traversable) are where the value is close to 255
    traversable_indices = np.where(img_array > 128)

    if traversable_indices[0].size == 0:
        print("Error: No white (traversable) pixels found in the image.")
        return None, None, None
    
    # Combine row (y) and column (x) indices into a list of (x, y) tuples
    traversable_points = list(zip(traversable_indices[1], traversable_indices[0]))
    
    # 3. Determine Start and Goal Points (x, y)
    
    # Start: Most Left (min x) AND Most Down (max y)
    start_point = min(traversable_points, key=lambda p: (-p[1], p[0])) 

    # Goal: Most Right (max x) AND Most Top (min y)
    goal_point = max(traversable_points, key=lambda p: (-p[1], p[0]))

    print(f"--- Map Generated ---")
    print(f"Grid Size: ({map_width}, {map_height})")
    print(f"Auto-Start Position (Left-Down): {start_point}")
    print(f"Auto-Goal Position (Right-Top): {goal_point}")

    # Identify obstacles (Black pixels)
    obstacle_indices = np.where(img_array < 128)
    obstacles_set = set(zip(obstacle_indices[1], obstacle_indices[0])) # (x, y) = (col, row)
    
    # 4. Setup and Run Planner
    
    # Create environment with the 500x500 dimensions
    print('Updating obstacles in the grid')
    env = pmp.Grid(map_width, map_height)
    env.update(obstacles_set)

    if start_point == goal_point:
        print("Error: Start and Goal points are the same. Cannot plan.")
        return None, None, None
    print('Starting the planner')
    planner = pmp.AStar(start=start_point, goal=goal_point, env=env)
    cost, path, expand = planner.plan()
    print('finished planning')
    
    # 5. Plot Result (Optional)
    if path:
        print(f"Path Found! Cost: {cost}")
        #planner.plot.animation(path, str(planner), cost, expand)
    else:
        print("No path found.")

    return cost, path, expand

# ==============================================================================
## Configuration and Execution
# ==============================================================================
base_path = '/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/a_start_generator/scripts/imgs/'
IMAGE_FILE = base_path + 'geo_wildcat_mosaic_mask.tif'

#OUTPUT_FILE = '50x50_grid_map.png' # <--- New output filename

# Execution: 
# 1. Load and Process Image
def main():

    try:
        img = Image.open(IMAGE_FILE).convert('L') # Convert to grayscale
    except FileNotFoundError:
        print(f"Error: Image file not found at {IMAGE_FILE}")
        return

    # Define target grid size
    target_height = 50
    target_width = 50
    OUTPUT_PATH_FILE = f'{IMAGE_FILE}_with_traj.png'

    # 1. Run the planner
    cost, path, expand = create_planner_from_image(IMAGE_FILE, img, target_height, target_width)

    if path is not None and len(path) > 0:
        # 2. Scale the grid coordinates back to the original image size
        w, h = img.size
        uw = w / target_width    # Unit Width (how many original pixels per grid cell)
        uh = h / target_height   # Unit Height (how many original pixels per grid cell)
        
        # B will hold the path coordinates scaled to the original image (x, y)
        # The (x + 0.5) and (y + 0.5) centers the line on the grid cell
        B = list()
        for (x, y) in path:
            # Note: The coordinates should be integer pairs for drawing
            B.append((int(uw * (x + 0.5)), int(uh * (y + 0.5))))

        # 3. Draw the path onto the original image
        
        # Create a draw object on a copy of the original image
        img_with_path = img.copy().convert("RGB") # Convert to RGB to draw a color line
        draw = ImageDraw.Draw(img_with_path)
        
        # Draw straight lines between consecutive points in B
        # The 'fill' color is red (255, 0, 0)
        # The 'width' (line thickness) is 3 pixels
        draw.line(B, fill=(255, 0, 0), width=3) 
        
         # Draw a small circle for EACH point in B (Yellow circles)
        point_radius = 2 # Radius for individual path points
        for p_x, p_y in B:
            draw.ellipse((p_x - point_radius, p_y - point_radius,
                        p_x + point_radius, p_y + point_radius), fill=(255, 255, 0)) # Yellow


        # Draw circles at the start and goal points for emphasis
        start_point_scaled = B[0]
        goal_point_scaled = B[-1]
        radius = 5
        
        # Start point (Green)
        draw.ellipse((start_point_scaled[0] - radius, start_point_scaled[1] - radius,
                    start_point_scaled[0] + radius, start_point_scaled[1] + radius), fill=(0, 255, 0))
        # Goal point (Blue)
        draw.ellipse((goal_point_scaled[0] - radius, goal_point_scaled[1] - radius,
                    goal_point_scaled[0] + radius, goal_point_scaled[1] + radius), fill=(0, 0, 255))
        
        # Draw Origin and Axes ---
        origin_x, origin_y = 0, 0 # The origin is at (0,0) in image coordinates
        origin_radius = 40
        axis_length = 200          # Length of the axis lines in pixels
        axis_width = 50            # Thickness of the axis lines
        axis_color_x = (255, 0, 0) # Red for X
        axis_color_y = (255, 255, 0) # Yellow for Y
        text_color = (255, 255, 255) # white text for contrast

        
        
        # Draw X-axis (Red line extending right from origin)
        x_axis_end = origin_x + axis_length + 5
        draw.line([(origin_x + 5, origin_y), (x_axis_end, origin_y)], fill=axis_color_x, width=axis_width)

        # Draw Y-axis (Yellow line extending down from origin)
        y_axis_end = origin_y + axis_length + 5
        draw.line([(origin_x, origin_y + 5), (origin_x, y_axis_end)], fill=axis_color_y, width=axis_width)
        
        # Goal point (Blue) - This now draws the Origin
        draw.ellipse((origin_x - origin_radius, origin_y - origin_radius,
                    origin_x + origin_radius, origin_y + origin_radius), fill=(0, 0, 255))
        
        # 2. Draw Axis Labels
        # 1. Load Font
        try:
            from PIL import ImageFont
            from matplotlib import font_manager 
            # Try to load a font; adjust path/name if necessary.
            # Find a common system font (e.g., DejaVu Sans, which Matplotlib often bundles)
            font_path = font_manager.findfont('DejaVu Sans', fallback_to_default=True)
            
            font = ImageFont.truetype(font_path, size=60)
        except IOError:
            print("Arial font not found. Using default font.")
            font = ImageFont.load_default()
        # X-axis label: Placed near the end of the X-axis line
        draw.text((x_axis_end + 10, origin_y + 10), "X", font=font, fill=text_color) 

        # Y-axis label: Placed near the end of the Y-axis line (to the right)
        draw.text((origin_x + 30, y_axis_end + 10), "Y", font=font, fill=text_color)
        
        # 4. Save the final image with the path
        try:
            img_with_path.save(OUTPUT_PATH_FILE)
            print(f"\nSaved final image with path to: {OUTPUT_PATH_FILE}")
        except IOError as e:
            print(f"Error saving path image: {e}")
            
    #breakpoint()

if __name__ == '__main__':
    main()
