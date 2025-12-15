# Optimized version for demo (faster collision checks and safer params).
from PIL import Image, ImageDraw
import numpy as np, math, heapq, os
from typing import List, Tuple, Optional

# Adjusted parameters for faster demo runs
PIXEL_STEP = 3
STEERING_SET = [-0.12, 0.0, 0.12]
NUM_TAU_SAMPLES = 2
FOOTPRINT_LENGTH = 40
FOOTPRINT_WIDTH = 20
HEADING_BINS = 24
GRID_RESOLUTION = 1
GOAL_TOLERANCE_POS = 8
GOAL_TOLERANCE_ANG = math.radians(20)
OUTPUT_DIR = "/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/output_dir/hybrid_a_start"
MAX_EXPANSIONS = 5000000
INFLATION = 0

class Pose:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

class Node:
    __slots__ = ("pose","g","f","parent","idx")
    def __init__(self, pose: Pose, g: float, f: float, parent: Optional[int]):
        self.pose = pose
        self.g = g
        self.f = f
        self.parent = parent
        self.idx = None

class HybridAStarPlanner:
    def __init__(self, mask_img_path: str):
        self.mask_img_path = mask_img_path
        self.occupancy, self.width, self.height = self._read_image(mask_img_path)
        self.expanded_nodes = []   # store all nodes visited for search tree-expansion visualization

    def _read_image(self, path: str):
        im = Image.open(path).convert("L")
        arr = np.array(im)
        #free = np.ones(arr.shape, dtype=bool)
        free = (arr > 250)
        h,w = arr.shape
        return free, w, h

    def _is_free_point(self, x: int, y: int, footprint: bool = False) -> bool:
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            if footprint: # footprint poligon can be outside map size
                return True
            return False
        return bool(self.occupancy[y, x])

    def _footprint_polygon(self, pose: Pose) -> List[Tuple[float,float]]:
        L = FOOTPRINT_LENGTH
        W = FOOTPRINT_WIDTH
        halfL = L/2.0
        halfW = W/2.0
        verts = [ ( halfL,  halfW),
                  ( halfL, -halfW),
                  (-halfL, -halfW),
                  (-halfL,  halfW) ]
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)
        world = []
        for vx, vy in verts:
            wx = pose.x + cos_t*vx - sin_t*vy
            wy = pose.y + sin_t*vx + cos_t*vy
            world.append((wx, wy))
        return world

    # Fast footprint collision test: check vertices, center, and midpoints only
    def _polygon_is_collision_free(self, polygon: List[Tuple[float,float]]) -> bool:
        test_points = polygon.copy()

        # midpoints
        n = len(polygon)
        for i in range(n):
            x0,y0 = polygon[i]
            x1,y1 = polygon[(i+1)%n]
            test_points.append(((x0+x1)/2.0, (y0+y1)/2.0))
        
        # Poligon's center point
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        cx = sum(xs)/len(xs)
        cy = sum(ys)/len(ys)
        test_points.append((cx, cy))

        for tx,ty in test_points:
            ix = int(round(tx))
            iy = int(round(ty))
            if not self._is_free_point(ix, iy, footprint=True):
                return False
        
        if not self._is_free_point(int(round(cx)), int(round(cy)), footprint=False):
            return False # Poligon's center is outside map
        return True

    def _primitive_collision_free(self, start_pose: Pose, kappa: float, length: float) -> bool:
        x0, y0, th0 = start_pose.x, start_pose.y, start_pose.theta
        N = max(2, NUM_TAU_SAMPLES)
        for i in range(1, N+1):
            s = (i/float(N)) * length
            if abs(kappa) < 1e-12:
                xs = x0 + s * math.cos(th0)
                ys = y0 + s * math.sin(th0)
                ths = th0
            else:
                ths = th0 + kappa * s
                xs = x0 + (math.sin(th0 + kappa*s) - math.sin(th0)) / kappa
                ys = y0 - (math.cos(th0 + kappa*s) - math.cos(th0)) / kappa
            p = Pose(xs, ys, ths)
            poly = self._footprint_polygon(p)
            if not self._polygon_is_collision_free(poly):
                return False
        return True

    def _integrate_primitive(self, start_pose: Pose, kappa: float, length: float) -> Pose:
        x0, y0, th0 = start_pose.x, start_pose.y, start_pose.theta
        if abs(kappa) < 1e-12:
            x1 = x0 + length * math.cos(th0)
            y1 = y0 + length * math.sin(th0)
            th1 = th0
        else:
            th1 = th0 + kappa * length
            x1 = x0 + (math.sin(th0 + kappa*length) - math.sin(th0)) / kappa
            y1 = y0 - (math.cos(th0 + kappa*length) - math.cos(th0)) / kappa
        th1 = (th1 + math.pi) % (2*math.pi) - math.pi
        return Pose(x1, y1, th1)

    def _discretize_pose(self, pose: Pose):
        ix = int(round(pose.x / GRID_RESOLUTION))
        iy = int(round(pose.y / GRID_RESOLUTION))
        itheta = int(math.floor(((pose.theta + math.pi) / (2*math.pi)) * HEADING_BINS)) % HEADING_BINS
        return (ix, iy, itheta)

    def _heuristic(self, pose: Pose, goal_pose: Pose) -> float:
        dx = pose.x - goal_pose.x
        dy = pose.y - goal_pose.y
        return math.hypot(dx, dy)

    def _analytic_connect(self, start_pose: Pose, goal_pose: Pose) -> Optional[List[Pose]]:
        max_horizon = 150
        s = Pose(start_pose.x, start_pose.y, start_pose.theta)
        path = [s]
        steering_abs = max(abs(k) for k in STEERING_SET)
        for step in range(max_horizon):
            vx = goal_pose.x - s.x
            vy = goal_pose.y - s.y
            dist = math.hypot(vx, vy)
            if dist < GOAL_TOLERANCE_POS:
                dtheta = abs(((goal_pose.theta - s.theta + math.pi) % (2*math.pi)) - math.pi)
                if dtheta <= GOAL_TOLERANCE_ANG:
                    return path + [goal_pose]
            desired_theta = math.atan2(vy, vx)
            heading_error = ((desired_theta - s.theta + math.pi) % (2*math.pi)) - math.pi
            desired_kappa = heading_error / max(1e-3, PIXEL_STEP)
            # keep desired_kappa is smaller than maximum kappa in steering set
            desired_kappa = max(min(desired_kappa, steering_abs), -steering_abs)
            # Get the kappa in STEERING_SET that is closest to desired_kappa
            kappa = min(STEERING_SET, key=lambda kk: abs(kk-desired_kappa))
            if not self._primitive_collision_free(s, kappa, PIXEL_STEP):
                return None
            newp = self._integrate_primitive(s, kappa, PIXEL_STEP)
            path.append(newp)
            s = newp
        return None

    def _reconstruct_path_pixels(self, nodes: List[Node], last_idx: int) -> List[Tuple[int,int]]:
        pts = []
        cur = last_idx
        while cur is not None:
            n = nodes[cur]
            pts.append((int(round(n.pose.x)), int(round(n.pose.y))))
            cur = n.parent
        pts.reverse()
        return pts

    def plan(self, start_pixel: Tuple[int,int], goal_pixel: Tuple[int,int], visualize: bool=True) -> List[Tuple[int,int]]:
        sx, sy = start_pixel
        gx, gy = goal_pixel
        start_theta = math.atan2(gy - sy, gx - sx)
        goal_theta = start_theta - math.pi/4
        start_pose = Pose(float(sx), float(sy), start_theta)
        goal_pose = Pose(float(gx), float(gy), goal_theta)
        print(f'start: {start_theta*180/math.pi}, goal: {goal_theta*180/math.pi}')
        if not self._is_free_point(sx, sy):
            raise RuntimeError("Start pixel is on an obstacle")
        if not self._is_free_point(gx, gy):
            raise RuntimeError("Goal pixel is on an obstacle")

        open_heap = []
        nodes: List[Node] = []
        g_best = {}

        def push_node(pose: Pose, g, parent_idx):
            h = self._heuristic(pose, goal_pose)
            f = g + h
            node = Node(pose, g, f, parent_idx)
            idx = len(nodes)
            node.idx = idx
            nodes.append(node)
            heapq.heappush(open_heap, (f, idx))
            return idx

        start_idx = push_node(start_pose, 0.0, None)
        g_best[self._discretize_pose(start_pose)] = 0.0

        expansions = 0
        closest_idx = start_idx
        closest_dist = self._heuristic(start_pose, goal_pose)

        while open_heap and expansions < MAX_EXPANSIONS:
            _, idx = heapq.heappop(open_heap)
            current = nodes[idx]
            expansions += 1
            
            self.expanded_nodes.append((int(round(current.pose.x)),
                                            int(round(current.pose.y))))

            # Keep track of closest node to goal
            dist_to_goal = self._heuristic(current.pose, goal_pose)
            if dist_to_goal < closest_dist:
                closest_dist = dist_to_goal
                closest_idx = idx

            analytic = self._analytic_connect(current.pose, goal_pose)
            if analytic is not None:
                parent = idx
                for p in analytic[1:]:
                    g_new = nodes[parent].g + PIXEL_STEP
                    new_idx = push_node(p, g_new, parent)
                    parent = new_idx
                path_pixels = self._reconstruct_path_pixels(nodes, parent)
                if visualize:
                    # 'parent' is the node index of last node appended when adding the analytic path in your existing code.
                    self._visualize(path_pixels, start_pixel, goal_pixel, nodes, path_last_node_idx=parent,
                                    visualize_tree=True, create_growth_gif=True)
                    self.visualize_tree(path_pixels)
                return path_pixels

            for kappa in STEERING_SET:
                if not self._primitive_collision_free(current.pose, kappa, PIXEL_STEP):
                    continue
                newpose = self._integrate_primitive(current.pose, kappa, PIXEL_STEP)
                g_new = current.g + PIXEL_STEP
                cell = self._discretize_pose(newpose)
                oldg = g_best.get(cell, float("inf"))
                if g_new + 1e-6 < oldg:
                    g_best[cell] = g_new
                    push_node(newpose, g_new, idx)
        print('Expansions: ', expansions)
        print('Path to goal not found, return path to closest reachable goal')
        # If goal not reached, return path to closest node
        path_pixels = self._reconstruct_path_pixels(nodes, closest_idx)
        if visualize:
            self._visualize(path_pixels, start_pixel, goal_pixel, nodes, path_last_node_idx=closest_idx,
                            visualize_tree=True, create_growth_gif=True)
            self.visualize_tree(path_pixels)
        return path_pixels


    
    def subsample_path(self, path_pixels, num_points):
        """
        Returns a list of num_points evenly spaced along path_pixels.
        """
        if len(path_pixels) <= num_points:
            # If path is short, just return the whole path
            return path_pixels

        # Evenly spaced indices along the path
        indices = np.linspace(0, len(path_pixels) - 1, num_points, dtype=int)
        subsampled = [path_pixels[i] for i in indices]
        return subsampled
    
    def _visualize3(self, path_pixels: List[Tuple[int,int]], start_pixel: Tuple[int,int], goal_pixel: Tuple[int,int], nodes: List[Node]):
        im = Image.open(self.mask_img_path).convert("RGB")
        draw = ImageDraw.Draw(im, "RGBA")
        if len(path_pixels) > 1:
            for p in path_pixels:
                draw.point(p, fill=(255,0,0,220))
                
        for i, (px,py) in enumerate(path_pixels[::max(1, len(path_pixels)//20)]):
        #for i, (px,py) in enumerate(self.subsample_path(path_pixels, 10)):
        #for i, (px,py) in enumerate([path_pixels[0], path_pixels[-1]]):
            if i*max(1, len(path_pixels)//20) < len(path_pixels)-1:
                nx, ny = path_pixels[min(len(path_pixels)-1, i*max(1, len(path_pixels)//20)+1)]
                theta = math.atan2(ny-py, nx-px)
            else:
                theta = 0.0
            p = Pose(px, py, theta)
            poly = self._footprint_polygon(p)
            draw.polygon(poly, outline=(0,192,0,60), fill=(0,192,0,60))
            draw.line((poly[0], poly[1]), (255, 0, 0, 100))
            draw.line((poly[2], poly[3]), (0, 0, 255, 100))
        sx, sy = start_pixel
        gx, gy = goal_pixel
        r = 1
        draw.ellipse((sx-r, sy-r, sx+r, sy+r), fill=(255,165,0,255))
        draw.ellipse((gx-r, gy-r, gx+r, gy+r), fill=(0,120,255,255))
        out_path = os.path.join(OUTPUT_DIR, f"hybridAStar_output.png")
        im.save(out_path)
        print(f"Visualization saved to: {out_path}")
     # ------------------- Visualization helpers -------------------
    def _draw_tree_static(self, im: Image.Image, nodes: List[Node], highlight_path_idxs: Optional[List[int]] = None):
        """
        Draws the complete expansion tree (edges from node to parent) onto the provided image.
        highlight_path_idxs: list of node indices forming the final path (optional) - these are drawn brighter.
        """
        draw = ImageDraw.Draw(im, "RGBA")

        # Draw tree edges: thin semi-transparent lines
        for i, node in enumerate(nodes):
            if node.parent is None:
                continue
            p = node.pose
            q = nodes[node.parent].pose
            x0, y0 = int(round(p.x)), int(round(p.y))
            x1, y1 = int(round(q.x)), int(round(q.y))

            # Color scaling: older nodes slightly darker
            alpha = 50  # base transparency
            # emphasize nodes that are in final path
            if highlight_path_idxs and i in highlight_path_idxs:
                color = (255, 100, 0, 200)
                width = 2
            else:
                color = (160, 160, 160, alpha)
                width = 1

            draw.line((x0, y0, x1, y1), fill=color, width=width)

        # Small dots for nodes (optional — keep subtle)
        for i, node in enumerate(nodes):
            x, y = int(round(node.pose.x)), int(round(node.pose.y))
            r = 1
            draw.ellipse((x-r, y-r, x+r, y+r), fill=(120,120,120,120))

    def _create_tree_growth_frames(self, nodes: List[Node], path_node_idx: Optional[int], start_pixel, goal_pixel,
                                   max_frames: int = 60):
        """
        Create a sequence of frames showing tree growth in node-creation order.
        Saves frames to OUTPUT_DIR and returns list of frame file paths.
        """
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        total_nodes = len(nodes)
        if total_nodes <= 1:
            return []

        # Determine snapshot indices (evenly spaced in creation order)
        num_snapshots = min(max_frames, total_nodes)
        snapshot_indices = np.linspace(1, total_nodes, num_snapshots, dtype=int)

        frame_paths = []
        # Precompute path node set if available
        path_set = set()
        if path_node_idx is not None:
            # reconstruct path node indices by following parents
            cur = path_node_idx
            while cur is not None:
                path_set.add(cur)
                cur = nodes[cur].parent

        for si_idx, upto in enumerate(snapshot_indices):
            im = Image.open(self.mask_img_path).convert("RGBA")
            draw = ImageDraw.Draw(im, "RGBA")

            # draw edges up to index 'upto'
            for i in range(1, upto):
                node = nodes[i]
                if node.parent is None or node.parent >= upto:
                    continue
                p = node.pose
                q = nodes[node.parent].pose
                x0, y0 = int(round(p.x)), int(round(p.y))
                x1, y1 = int(round(q.x)), int(round(q.y))

                # color depends on whether part of final path (but final path nodes may not be in early frames)
                if i in path_set:
                    color = (255, 0, 0, 210)
                    width = 2
                else:
                    color = (180, 180, 180, 120)
                    width = 1

                draw.line((x0, y0, x1, y1), fill=color, width=width)

            # draw start/goal
            sx, sy = start_pixel
            gx, gy = goal_pixel
            r = 2 + int(2*si_idx/len(snapshot_indices))
            draw.ellipse((sx-r, sy-r, sx+r, sy+r), fill=(255,165,0,255))
            draw.ellipse((gx-r, gy-r, gx+r, gy+r), fill=(0,120,255,255))
            frames_dir = os.path.join(OUTPUT_DIR, 'frames')
            os.makedirs(frames_dir, exist_ok=True)
            frame_path = os.path.join(frames_dir, f"hybridAStar_tree_frame_{si_idx:03d}.png")
            im.save(frame_path)
            frame_paths.append(frame_path)

        # Optionally make GIF (if PIL supports) - limited frames to avoid huge files
        if len(frame_paths) > 1:
            try:
                imgs = [Image.open(p).convert("RGBA") for p in frame_paths]
                gif_path = os.path.join(OUTPUT_DIR, "hybridAStar_tree_growth.gif")
                imgs[0].save(gif_path, save_all=True, append_images=imgs[1:], duration=120, loop=0)
                print(f"Animated growth GIF saved to: {gif_path}")
            except Exception as e:
                print("Could not create GIF:", e)

        return frame_paths

    def _visualize(self, path_pixels: List[Tuple[int,int]], start_pixel: Tuple[int,int],
                   goal_pixel: Tuple[int,int], nodes: List[Node], path_last_node_idx: Optional[int]=None,
                   visualize_tree: bool = True, create_growth_gif: bool = True):
        """
        Visualize the final path and (optionally) the search tree.
        - path_pixels: list of (x,y) pixels of path
        - nodes: full list of Node objects created during planning (order = creation order)
        - path_last_node_idx: index of the final node in `nodes` that corresponds to the path's last element (if any)
        - visualize_tree: draw the full tree as a static overlay
        - create_growth_gif: create a set of frames and a gif showing progressive growth
        """
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        im = Image.open(self.mask_img_path).convert("RGBA")
        draw = ImageDraw.Draw(im, "RGBA")

        # --- draw tree (static full overlay) ---
        if visualize_tree:
            # make a copy for the tree overlay so color intensities are readable
            tree_im = im.copy()
            self._draw_tree_static(tree_im, nodes, highlight_path_idxs=None)
            im = Image.alpha_composite(im, tree_im)  # combine
            draw = ImageDraw.Draw(im, "RGBA")

        # --- draw path pixels (prominent) ---
        if len(path_pixels) > 1:
            # draw thicker red polyline for the path
            draw.line(path_pixels, fill=(255, 0, 0, 220), width=2)
            # draw red points
            for p in path_pixels:
                draw.point(p, fill=(255,0,0,220))

        # --- draw sampled footprints along path (like before) ---
        sample_n = min(30, max(4, len(path_pixels)//max(1, len(path_pixels)//30)))
        sampled = self.subsample_path(path_pixels, sample_n)
        for i, (px,py) in enumerate(sampled):
            # compute heading from the next sample when available
            if i < len(sampled)-1:
                nx, ny = sampled[i+1]
                theta = math.atan2(ny-py, nx-px)
            else:
                theta = 0.0
            p = Pose(px, py, theta)
            poly = self._footprint_polygon(p)
            draw.polygon(poly, outline=(0,192,0,180), fill=(0,192,0,60))
            # optionally draw direction line
            if len(poly) > 0:
                draw.line((poly[0], poly[1]), fill=(0,120,0,200))

        # start and goal markers
        sx, sy = start_pixel
        gx, gy = goal_pixel
        r = 3
        draw.ellipse((sx-r, sy-r, sx+r, sy+r), fill=(255,165,0,255))
        draw.ellipse((gx-r, gy-r, gx+r, gy+r), fill=(0,120,255,255))

        # Save final visualization
        out_path = os.path.join(OUTPUT_DIR, f"hybridAStar_output.png")
        im.convert("RGB").save(out_path)
        print(f"Visualization saved to: {out_path}")

        # --- create growth frames/gif if requested ---
        if create_growth_gif:
            # path_last_node_idx is used to highlight the final path segments in the frames (if known)
            frame_paths = self._create_tree_growth_frames(nodes, path_last_node_idx, start_pixel, goal_pixel)
            if frame_paths:
                print(f"Saved {len(frame_paths)} growth frames to {OUTPUT_DIR}")

    def visualize_tree(self, path_pixels, out_path="tree_expansion.png"):
        """
        Draw:
        - Greyscale map
        - Blue: all expanded nodes
        - Yellow: final planned path
        - Green: start
        - Red: goal
        """
        img = Image.fromarray((self.occupancy * 255).astype(np.uint8)).convert("RGB")
        draw = ImageDraw.Draw(img)

        # Draw expanded nodes
        for (x, y) in self.expanded_nodes:
            draw.point((x, y), fill=(80, 80, 255))  # blue

        # Draw path
        for i in range(len(path_pixels) - 1):
            draw.line([path_pixels[i], path_pixels[i+1]], fill=(255, 255, 0), width=2)

        # Start, Goal
        if len(path_pixels) > 0:
            draw.ellipse([path_pixels[0][0]-3, path_pixels[0][1]-3,
                        path_pixels[0][0]+3, path_pixels[0][1]+3], fill=(0,255,0))
            draw.ellipse([path_pixels[-1][0]-3, path_pixels[-1][1]-3,
                        path_pixels[-1][0]+3, path_pixels[-1][1]+3], fill=(255,0,0))

        img.save(out_path)
        print(f"Tree expansion visualization saved to {out_path}")

def hybridAStart(binaryMaskPngImage: str, start_pixel: Tuple[int,int], goal_pixel: Tuple[int,int], visualize: bool=True) -> List[Tuple[int,int]]:
    planner = HybridAStarPlanner(binaryMaskPngImage)
    path = planner.plan(start_pixel, goal_pixel, visualize=visualize)
    return path

def _create_demo_map(path="/mnt/data/demo_mask.png"):
    W, H = 200, 200
    im = Image.new("L", (W,H), color=255)
    draw = ImageDraw.Draw(im)
    draw.rectangle((40, 0, 60, 140), fill=0)
    draw.rectangle((100, 60, 160, 80), fill=0)
    draw.ellipse((130, 120, 160, 150), fill=0)
    im.save(path)
    return path

import cv2

def get_start_goal_interactive(img_path):
    """
    Opens the image and waits for two clicks:
    1. Start Point (Green circle)
    2. Goal Point (Blue circle)
    Press any key to confirm selection after two clicks.
    """
    img = cv2.imread(img_path)
    if img is None:
        raise RuntimeError(f"Could not read image: {img_path}")
        
    points = []
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 2:
                points.append((x, y))
                # Visual feedback
                color = (0, 255, 0) if len(points) == 1 else (255, 0, 0) # Green for Start, Blue for Goal
                cv2.circle(img, (x, y), 5, color, -1)
                cv2.imshow("Select Start (Green) then Goal (Blue)", img)
                print(f"Selected point {len(points)}: {(x, y)}")

    cv2.imshow("Select Start (Green) then Goal (Blue)", img)
    cv2.setMouseCallback("Select Start (Green) then Goal (Blue)", mouse_callback)
    
    print("Please click the START point (Green) and then the GOAL point (Blue) on the image window.")
    print("Press any key after selecting both points to continue...")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    if len(points) < 2:
        raise RuntimeError("Two points were not selected.")
        
    return points[0], points[1]

demo_path = _create_demo_map(path="/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/output_dir/astart_planner/frame_1764638009_astart_planner.png")
demo_path = "/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/input_imgs/ground_truth_gazebo_BEV_binary.png"

print(f"Opening interactive selection for {demo_path}...")
try:
    start, goal = get_start_goal_interactive(demo_path)
    print(f"Selected Start: {start}, Goal: {goal}")
    
    print("Running hybrid A* on demo map... (fast-mode)")
    result_path = hybridAStart(demo_path, start, goal, visualize=True)
    print("Path length (pixels):", len(result_path))
    print("Sample path points (first 10):", result_path[:10])
except Exception as e:
    print("Planner failed:", str(e))
    import traceback
    traceback.print_exc()
    
print(f"Saved images (mask and planner output) are in {OUTPUT_DIR}")
