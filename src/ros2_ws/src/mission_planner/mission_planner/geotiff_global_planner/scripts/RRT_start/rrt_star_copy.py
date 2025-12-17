# RRT* planner adapted from your Hybrid A* file
# Keeps visualization, GIF generation, interactive start/goal and image I/O logic.
from PIL import Image, ImageDraw
import numpy as np, math, os, random, heapq
from typing import List, Tuple, Optional
import time

# ---------------- RRT* PARAMETERS (kept together for easy tuning) ----------------
RRT_MAX_ITERS = 20000           # maximum number of sampling iterations
RRT_STEP = 12                   # step length in pixels when steering toward sample
RRT_GOAL_SAMPLE_RATE = 0.07     # probability of sampling the goal directly
RRT_REWIRE_RADIUS = 40.0        # neighbourhood radius for rewiring (pixels)
RRT_GOAL_TOL = 10               # distance tolerance (pixels) to consider goal reached
RRT_COLLISION_SAMPLES = 6       # number of intermediate checks along an edge
RRT_MIN_DISTANCE_BETWEEN_NODES = 1.0  # avoid duplicate nodes near each other
# -------------------------------------------------------------------------------

FOOTPRINT_LENGTH = 200
FOOTPRINT_WIDTH = 250
OUTPUT_DIR = "/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/output_dir/hybrid_a_start"
class Pose:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

# Slightly different Node class for RRT* but keeping the same spirit/fields
class RRTNode:
    __slots__ = ("pose","cost","parent","idx")
    def __init__(self, pose: Pose, cost: float = 0.0, parent: Optional[int] = None):
        self.pose = pose
        self.cost = cost
        self.parent = parent
        self.idx = None

import numpy as np
import math
import random
from scipy.spatial import cKDTree

class RRTStarPlanner:
    def __init__(self, mask_img_path: str, grid=None):
        self.mask_img_path = mask_img_path
        _, self.occupancy, self.width, self.height = self.__read_image(mask_img_path, grid)
        self.expanded_nodes = []   # nodes for visualization
        self.node_xy = np.empty((0, 2), dtype=np.float32)  # positions for KD-tree
        self.kdtree = None
        print(f"[RRT*] Occupancy grid(w, h): {self.width, self.height}")

    # ------------------ Image loading ------------------
    def __read_image(self, path: str, grid=None):
        if isinstance(path, str):
            from PIL import Image
            im = Image.open(path).convert("L")
            img_arr = np.array(im)
        elif isinstance(path, np.ndarray):
            img_arr = path
            if img_arr.ndim == 3:
                img_arr = img_arr.mean(axis=2)
        else:
            raise TypeError(f"Unsupported type for path: {type(path)}")
        free = (img_arr > 250) if grid is None else (grid != np.inf)
        h,w = free.shape
        return img_arr, free, w, h

    # ------------------ Collision checks ------------------
    def _is_free_point(self, x: int, y: int, footprint: bool = False) -> bool:
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return footprint  # allow footprint outside
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
    
    # ------------------ Node insertion ------------------
    def _push_node(self, nodes, pose, cost, parent_idx):
        idx = len(nodes)
        node = RRTNode(pose, cost, parent_idx)
        node.idx = idx
        nodes.append(node)
        self.expanded_nodes.append((pose.x, pose.y))
        # update KD-tree
        self.node_xy = np.vstack((self.node_xy, [pose.x, pose.y]))
        self.kdtree = cKDTree(self.node_xy)
        return idx

    # ------------------ Nearest neighbor using KD-tree ------------------
    def _nearest_node_idx(self, point):
        if self.kdtree is None:
            raise RuntimeError("KD-tree not initialized")
        _, idx = self.kdtree.query(point)
        return idx

    # ------------------ Nearby nodes within radius ------------------
    def _nearby_nodes(self, idx: int, radius: float) -> list[int]:
        if self.kdtree is None:
            return []
        point = self.node_xy[idx]
        indices = self.kdtree.query_ball_point(point, radius)
        return indices

    # ------------------ Check if too close to existing node ------------------
    def _near_existing_node(self, pose, min_dist: float) -> bool:
        if self.kdtree is None or len(self.node_xy) == 0:
            return False
        # squared distance check for speed
        idxs = self.kdtree.query_ball_point([pose.x, pose.y], r=min_dist)
        return len(idxs) > 0

    # ------------------ Steer towards target ------------------
    def _steer_towards(self, from_pose: Pose, to_point: tuple[int,int], step: float) -> Pose:
        dx = to_point[0] - from_pose.x
        dy = to_point[1] - from_pose.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return Pose(from_pose.x, from_pose.y, from_pose.theta)
        scale = min(step / dist, 1.0)
        nx = from_pose.x + dx * scale
        ny = from_pose.y + dy * scale
        theta = math.atan2(dy, dx)
        return Pose(nx, ny, theta)

    def _polygon_is_collision_free(self, polygon: list[tuple[float,float]]) -> bool:
        """Vectorized collision check for a polygon using midpoint + center + vertices"""

        polygon = np.array(polygon)  # shape (N,2)
        n = len(polygon)

        # --- midpoints ---
        next_idx = np.roll(np.arange(n), -1)
        midpoints = (polygon + polygon[next_idx]) / 2.0  # shape (N,2)

        # --- center ---
        center = polygon.mean(axis=0, keepdims=True)  # shape (1,2)

        # --- all points to test ---
        test_points = np.vstack([polygon, midpoints, center])  # shape (N*2+1, 2)
        ix = np.clip(np.round(test_points[:,0]).astype(int), 0, self.width-1)
        iy = np.clip(np.round(test_points[:,1]).astype(int), 0, self.height-1)

        # occupancy check in one go
        free = self.occupancy[iy, ix]

        # If any test point is not free, collision!
        if not free.all():
            return False

        # final center check (must be strictly inside map)
        cx, cy = int(round(center[0,0])), int(round(center[0,1]))
        if not self._is_free_point(cx, cy, footprint=False):
            return False

        return True

    def _edge_collision_free(self, a: Pose, b: Pose, samples: int = RRT_COLLISION_SAMPLES) -> bool:
        """Vectorized edge collision check"""
        if samples <= 0:
            return True

        # --- linear interpolation ---
        ts = np.linspace(0, 1, samples+1)[1:]  # skip t=0 (already a)
        xs = a.x + (b.x - a.x) * ts
        ys = a.y + (b.y - a.y) * ts

        # Compute headings along edge
        dx = b.x - a.x
        dy = b.y - a.y
        thetas = np.arctan2(dy, dx) * np.ones_like(xs)

        # --- batch check each pose ---
        for x, y, theta in zip(xs, ys, thetas):
            p = Pose(x, y, theta)
            poly = self._footprint_polygon(p)
            if not self._polygon_is_collision_free(poly):
                return False

        return True
    
    def _reconstruct_path_pixels(self, nodes: List[RRTNode], last_idx: int) -> List[Tuple[int,int]]:
            pts = []
            cur = last_idx
            while cur is not None:
                n = nodes[cur]
                pts.append((int(round(n.pose.x)), int(round(n.pose.y))))
                cur = n.parent
            pts.reverse()
            return pts
    
    def plan(self, start_pixel: Tuple[int,int], goal_pixel: Tuple[int,int], visualize: bool=True) -> list[tuple[int,int]]:
        """RRT* planner with KD-tree optimization for nearest neighbor search"""
        import time
        start_time = time.perf_counter()
        
        sx, sy = start_pixel
        gx, gy = goal_pixel
        start_theta = math.atan2(gy - sy, gx - sx)
        start_pose = Pose(float(sx), float(sy), start_theta)
        goal_theta = start_theta - math.pi/4
        goal_pose = Pose(float(gx), float(gy), goal_theta)

        # Check start and goal are free
        if not self._is_free_point(sx, sy):
            raise RuntimeError("Start pixel is on an obstacle")
        if not self._is_free_point(gx, gy):
            raise RuntimeError("Goal pixel is on an obstacle")

        nodes: list[RRTNode] = []

        # Node insertion function
        def push_node(pose: Pose, cost: float, parent_idx: int | None) -> int:
            return self._push_node(nodes, pose, cost, parent_idx)

        # Insert start node
        start_idx = push_node(start_pose, 0.0, None)
        closest_idx = start_idx
        closest_dist = math.hypot(start_pose.x - gx, start_pose.y - gy)
        reached_goal = False
        goal_node_idx = None

        for it in range(RRT_MAX_ITERS):
            # ------------------ Sample ------------------
            if random.random() < RRT_GOAL_SAMPLE_RATE:
                rx, ry = gx, gy
            else:
                rx = random.randint(0, self.width-1)
                ry = random.randint(0, self.height-1)

            if not self._is_free_point(rx, ry):
                continue

            # ------------------ Nearest node ------------------
            nearest_idx = self._nearest_node_idx((rx, ry))
            nearest = nodes[nearest_idx]

            # ------------------ Steer ------------------
            new_pose = self._steer_towards(nearest.pose, (rx, ry), RRT_STEP)

            # Skip if too close to existing nodes
            if self._near_existing_node(new_pose, RRT_MIN_DISTANCE_BETWEEN_NODES):
                continue

            # ------------------ Collision check ------------------
            if not self._edge_collision_free(nearest.pose, new_pose, samples=RRT_COLLISION_SAMPLES):
                continue

            new_cost = nearest.cost + math.hypot(new_pose.x - nearest.pose.x, new_pose.y - nearest.pose.y)
            new_idx = push_node(new_pose, new_cost, nearest_idx)

            # ------------------ Rewiring ------------------
            neighbor_idxs = self._nearby_nodes(new_idx, RRT_REWIRE_RADIUS)
            best_parent = nodes[new_idx].parent
            best_cost = nodes[new_idx].cost

            # Find best parent among neighbors
            for ni in neighbor_idxs:
                if ni == new_idx:
                    continue
                nnode = nodes[ni]
                d = math.hypot(new_pose.x - nnode.pose.x, new_pose.y - nnode.pose.y)
                candidate_cost = nnode.cost + d
                if candidate_cost + 1e-8 < best_cost:
                    if self._edge_collision_free(nnode.pose, new_pose, samples=RRT_COLLISION_SAMPLES):
                        best_cost = candidate_cost
                        best_parent = ni

            nodes[new_idx].parent = best_parent
            nodes[new_idx].cost = best_cost

            # Rewire neighbors to use new node if shorter
            for ni in neighbor_idxs:
                if ni == new_idx:
                    continue
                nnode = nodes[ni]
                d = math.hypot(nnode.pose.x - new_pose.x, nnode.pose.y - new_pose.y)
                candidate_cost = nodes[new_idx].cost + d
                if candidate_cost + 1e-8 < nnode.cost:
                    if self._edge_collision_free(new_pose, nnode.pose, samples=RRT_COLLISION_SAMPLES):
                        nnode.parent = new_idx
                        nnode.cost = candidate_cost

            # ------------------ Update closest node to goal ------------------
            dist_to_goal = math.hypot(new_pose.x - gx, new_pose.y - gy)
            if dist_to_goal < closest_dist:
                closest_dist = dist_to_goal
                closest_idx = new_idx

            # ------------------ Check if goal reached ------------------
            if dist_to_goal <= RRT_GOAL_TOL:
                goal_connect_pose = Pose(float(gx), float(gy), math.atan2(gy - new_pose.y, gx - new_pose.x))
                if self._edge_collision_free(new_pose, goal_connect_pose, samples=RRT_COLLISION_SAMPLES):
                    goal_idx = push_node(goal_connect_pose, nodes[new_idx].cost + dist_to_goal, new_idx)
                    reached_goal = True
                    goal_node_idx = goal_idx
                    print(f"[RRT*] Goal reached in {it+1} iterations (nodes: {len(nodes)})")
                    break

        # ------------------ Reconstruct path ------------------
        if reached_goal and goal_node_idx is not None:
            path_pixels = self._reconstruct_path_pixels(nodes, goal_node_idx)
            path_last_node_idx = goal_node_idx
        else:
            print(f"[RRT*] Did not reach exact goal, returning path to closest node")
            path_pixels = self._reconstruct_path_pixels(nodes, closest_idx)
            path_last_node_idx = closest_idx

        end_time = time.perf_counter()
        print(f"[RRT*] Path found in {(end_time - start_time)*1000:.2f} ms")

        # ------------------ Visualization ------------------
        if visualize:
            self._visualize(
                path_pixels,
                start_pixel,
                goal_pixel,
                nodes,
                path_last_node_idx=path_last_node_idx,
                visualize_tree=True,
                create_growth_gif=True
            )

        return path_pixels

    
    # ------------------- Visualization helpers (copied & adapted from original) -------------------
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

    def _draw_tree_static(self, im: Image.Image, nodes: List[RRTNode], highlight_path_idxs: Optional[List[int]] = None):
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

    def draw_footprint_at(self, p: Pose, im, draw):
        def draw_transparent_polygon(im, poly, fill, outline=None):
            # Helper function to enable transparency when using PIL
            overlay = Image.new("RGBA", im.size, (0,0,0,0))
            ImageDraw.Draw(overlay).polygon(poly, fill=fill, outline=outline)
            im.paste(overlay, mask=overlay)
            return im
        
        poly = self._footprint_polygon(p)
        draw_transparent_polygon(im, poly, outline=(0,192,0,180), fill=(0,192,0,60))
        # optionally draw direction line
        if len(poly) > 0:
            draw.line((poly[0], poly[1]), fill=(0,0,255,200))

    def _create_tree_growth_frames(self, nodes: List[RRTNode], path_node_idx: Optional[int], start_pixel, goal_pixel,
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
            if isinstance(self.mask_img_path, str):
                im = Image.open(self.mask_img_path).convert("RGBA")
            else:
                im = Image.fromarray(self.mask_img_path.astype(np.uint8), mode="RGB").convert("RGBA")
            
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
                    #self.draw_footprint_at(p, im, draw)
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
            frame_path = os.path.join(frames_dir, f"rrtstar_tree_frame_{si_idx:03d}.png")
            im.save(frame_path)
            frame_paths.append(frame_path)

        # Optionally make GIF (if PIL supports) - limited frames to avoid huge files
        if len(frame_paths) > 1:
            try:
                imgs = [Image.open(p).convert("RGBA") for p in frame_paths]
                gif_path = os.path.join(OUTPUT_DIR, "rrtstar_tree_growth.gif")
                imgs[0].save(gif_path, save_all=True, append_images=imgs[1:], duration=120, loop=0)
                print(f"Animated growth GIF saved to: {gif_path}")
            except Exception as e:
                print("Could not create GIF:", e)

        return frame_paths

    def _visualize(self, path_pixels: List[Tuple[int,int]], start_pixel: Tuple[int,int],
                   goal_pixel: Tuple[int,int], nodes: List[RRTNode], path_last_node_idx: Optional[int]=None,
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
        if isinstance(self.mask_img_path, str):
            im = Image.open(self.mask_img_path).convert("RGBA")
        else:
            im = Image.fromarray(self.mask_img_path.astype(np.uint8), mode="RGB").convert("RGBA")
        
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
            draw.line(path_pixels, fill=(255, 0, 0, 220), width=1)
            # draw red points
            for p in path_pixels:
                #draw.point(p, fill=(255,0,0,220))
                px, py = p
                r = 2
                draw.ellipse((px-r, py-r, px+r, py+r), fill=(0,0,165,255))
                

        # --- draw sampled footprints along path (like before) ---
        sampled = path_pixels[::max(1, len(path_pixels)//20)]

        def draw_transparent_polygon(im, poly, fill, outline=None):
            # Helper function to enable transparency when using PIL
            overlay = Image.new("RGBA", im.size, (0,0,0,0))
            ImageDraw.Draw(overlay).polygon(poly, fill=fill, outline=outline)
            im.paste(overlay, mask=overlay)
            return im
        
        for i, (px,py) in enumerate(sampled):
            # compute heading from the next sample when available
            if i < len(sampled)-1:
                nx, ny = sampled[i+1]
                theta = math.atan2(ny-py, nx-px)
            else:
                theta = 0.0
            p = Pose(px, py, theta)
            poly = self._footprint_polygon(p)
            draw_transparent_polygon(im, poly, outline=(0,192,0,180), fill=(0,192,0,60))
            
            # optionally draw direction line
            if len(poly) > 0:
                draw.line((poly[0], poly[1]), fill=(0,0,255,200))

        # start and goal markers
        sx, sy = start_pixel
        gx, gy = goal_pixel
        r = 3
        draw.ellipse((sx-r, sy-r, sx+r, sy+r), fill=(255,165,0,255))
        draw.ellipse((gx-r, gy-r, gx+r, gy+r), fill=(0,120,255,255))

        # Save final visualization
        out_path = os.path.join(OUTPUT_DIR, f"rrtstar_output.png")
        im.convert("RGB").save(out_path)
        print(f"Visualization saved to: {out_path}")

        # --- create growth frames/gif if requested ---
        if create_growth_gif:
            frame_paths = self._create_tree_growth_frames(nodes, path_last_node_idx, start_pixel, goal_pixel)
            if frame_paths:
                print(f"Saved {len(frame_paths)} growth frames to {OUTPUT_DIR}")

    def visualize_tree(self, path_pixels, out_path="rrtstar_tree_expansion.png"):
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

# --------------- Public entry point that mirrors your original style ----------------
def rrtStarPlan(binaryMaskPngImage: str, start_pixel: Tuple[int,int], goal_pixel: Tuple[int,int], visualize: bool=True) -> List[Tuple[int,int]]:
    planner = RRTStarPlanner(binaryMaskPngImage)
    path = planner.plan(start_pixel, goal_pixel, visualize=visualize)
    return path

# ---------------- demo / interactive code (keeps your original behaviour) ----------------
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


if __name__ == "__main__":
    #demo_path = _create_demo_map(path="/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/output_dir/astart_planner/frame_1764638009_astart_planner.png")
    demo_path = "/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/output_dir/depth_pipeline/frame_2/frame_2_bev_binary_inpainting.png"

    print(f"Opening interactive selection for {demo_path}...")
    try:
        start, goal = get_start_goal_interactive(demo_path)
        print(f"Selected Start: {start}, Goal: {goal}")
        
        print("Running RRT* on demo map... (fast-mode)")
        result_path = rrtStarPlan(demo_path, start, goal, visualize=True)
        print("Path length (pixels):", len(result_path))
    except Exception as e:
        print("Planner failed:", str(e))
        import traceback
        traceback.print_exc()
        
    print(f"Output dir is: {OUTPUT_DIR}")
