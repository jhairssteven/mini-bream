import os
import numpy as np
import torch
from PIL import Image


# Depth anything dependencies
import argparse
import cv2
import glob
import numpy as np
import open3d as o3d
import os
from PIL import Image
import torch
import matplotlib

from metric_depth.depth_anything_v2.dpt import DepthAnythingV2

class DepthModel():
    def __init__(self, args):
        DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }
        
        depth_anything = DepthAnythingV2(**model_configs[args.encoder])
        depth_anything.load_state_dict(torch.load(args.load_from, map_location='cpu', weights_only=True))
        self.depth_anything = depth_anything.to(DEVICE).eval()

        # BEV projection data
        self.x_min = 0; self.z_min = 0; self.cell_size = 0; self.height = 0

    def get_depth(self, filename, image_input, mask, args):

        # Read the image using OpenCV
        raw_image = image_input.copy()
        height, _, _ = raw_image.shape
        raw_image[mask == 0] = [255.0, 255.0, 255.0]

        depth_pred = self.depth_anything.infer_image(raw_image, height)
        
        #overlay[mask == 1] = (0.4 * overlay[mask == 1] + 0.6 * color).astype(np.uint8)
        output_base = os.path.splitext(os.path.basename(filename))[0]
        pcd = self.as_pcl(depth_pred, args, raw_image, output_base, mask)
        bev_image_vis, bev_binary_image_uint8, bev_filename, bev_image_binary_inpainted_uint8 = self.pcl_to_BEV(pcd, output_base, args, raw_image)
        
        print(f'[{output_base}] [DA2] Estimated depth (min, max): {round(depth_pred.min(), 3)} (m), {round(depth_pred.max(), 3)} (m)')
        
        depth_pred = (depth_pred - depth_pred.min()) / (depth_pred.max() - depth_pred.min()) * 255.0
        depth = depth_pred.astype(np.uint8)
        
        if args.grayscale:
            depth = np.repeat(depth[..., np.newaxis], 3, axis=-1)
        else:
            cmap = matplotlib.colormaps.get_cmap('Spectral')
            depth = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
        
        base_name = os.path.splitext(os.path.basename(filename))[0]
        depth_filename = base_name + '_depth.png'
        output_path = os.path.join(args.outdir, base_name, depth_filename)
        if args.pred_only:
            if args.write_singles:
                cv2.imwrite(output_path, depth)
        else:
            split_region = np.ones((raw_image.shape[0], 50, 3), dtype=np.uint8) * 255
            combined_result = cv2.hconcat([raw_image, split_region, depth])
            if args.write_singles:
                cv2.imwrite(output_path, combined_result)

        return depth_pred, bev_image_vis, bev_binary_image_uint8, bev_filename, bev_image_binary_inpainted_uint8, depth, depth_filename, pcd
    
    def as_pcl(self, pred, args, input_image, output_base, water_mask):
        color_image = cv2.cvtColor(input_image, cv2.COLOR_RGB2BGR)
        height, width, _ = color_image.shape
        # Resize depth prediction to match the original image size
        resized_pred = Image.fromarray(pred).resize((width, height), Image.NEAREST)

        # Generate mesh grid and calculate point cloud coordinates
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        x = (x - width / 2) / args.focal_length_x
        y = (y - height / 2) / args.focal_length_y
        z = np.array(resized_pred)

        ## Apply the water mask
        # Flatten arrays
        x = x.flatten()
        y = y.flatten()
        z = z.flatten()
        mask_flat = water_mask.flatten()

        # Filter points by water mask
        x = x[mask_flat == 1]
        y = y[mask_flat == 1]
        z = z[mask_flat == 1]
        
        points = np.stack((np.multiply(x, z), np.multiply(y, z), z), axis=-1).reshape(-1, 3)
        
        colors = np.array(color_image).reshape(-1, 3) / 255.0
        colors = colors[mask_flat == 1]
        
        # Create the point cloud and save it to the output directory
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if args.save_pcl:
            pcl_outdir = os.path.join(args.outdir, 'as_pcl')
            os.makedirs(pcl_outdir, exist_ok=True)
            o3d.io.write_point_cloud(os.path.join(pcl_outdir, output_base + ".ply"), pcd)        
        return pcd
    
    def bev_binary_inpainting(self, bev_binary):
        # Normalize for mask operations (non-zero = valid)
        bev_binary_bool = (bev_binary > 0).astype(np.uint8)

        # Apply morphological closing to fill gaps
        kernel = np.ones((10, 1), np.uint8)   # increase size for thicker fill
        bev_binary_inpainted = cv2.morphologyEx(bev_binary_bool, cv2.MORPH_CLOSE, kernel)
        return bev_binary_inpainted

    def pcl_to_BEV(self, pcd, output_base, args, color_image):
        """ bev_image_vis: (h,w,3): Three channel image each channel [0, 255] of the BEV representation of the pcd.
            bev_image_binary: (h, w): Single channel [0, 1] with black foreground."""
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)  # RGB in [0,1]

        # --- Remove white points (background) ---
        # A tolerance helps because colors may not be exactly 1.0
        ##white_tolerance = 0.98
        ##non_white_mask = ~(np.all(colors > white_tolerance, axis=1))
        ##points = points[non_white_mask]
        ##colors = colors[non_white_mask]

        # --- Select axes for BEV projection ---
        # X: lateral (right-left), Z: depth (forward)
        x = points[:, 0]
        z = points[:, 2]

        # --- Optional filtering to limit range ---
        x_min, x_max = np.min(x), np.max(x)    # meters
        z_min, z_max = np.min(z), np.max(z)    # meters in front of camera
        ##mask = (x > x_min) & (x < x_max) & (z > z_min) & (z < z_max)
        ##print(x_min, x_max, z_min, z_max)
        ##x, z, colors = x[mask], z[mask], colors[mask]

        # --- Define BEV map resolution ---
        cell_size = 0.01  # meters per pixel (higher = coarser)
        width = int((x_max - x_min) / cell_size)
        height = int((z_max - z_min) / cell_size)

        # Save projection properties to perform inverse mappint if necessary
        self.x_min = x_min; self.z_min = z_min; self.cell_size = cell_size; self.height = height
        
        bev_bgr_color = 1.0 # White
        bev_image = np.ones((height, width, 3), dtype=np.float32) * bev_bgr_color
        
        bev_bgr_color_binary = 0.0 # Black
        bev_image_binary = np.zeros((height, width), dtype=np.float32) * bev_bgr_color_binary

        # --- Convert to pixel coordinates ---
        u = ((x - x_min) / cell_size).astype(np.int32)
        v = ((z - z_min) / cell_size).astype(np.int32)

        # Fill pixels (color map)
        for i in range(len(u)):
            # check if u[i] and v[i] are not pixels for x_max and z_max respectively
            if 0 <= u[i] < width and 0 <= v[i] < height:
                #water_color = [1.0, 1.0, 1.0] if args.bev_as_binary_mask else colors[i]
                bev_image[height - v[i] - 1, u[i], :] = colors[i]
                bev_image_binary[height - v[i] - 1, u[i]] = 1.0 # White color for traversable pixels

        # --- Optional smoothing ---
        bev_image = cv2.GaussianBlur(bev_image, (3, 3), 0)

        ci_height, ci_width, _ = color_image.shape
        bev_image_vis = (bev_image[:, :, ::-1] * 255).astype(np.uint8)
        bev_filename = f"{output_base}_bev"
        if args.pred_only:
            output_path = os.path.join(args.outdir, output_base)
            
            # Save BEV colored image only
            if args.write_singles:
                cv2.imwrite(os.path.join(output_path, f'{bev_filename}_colored.png', ), bev_image_vis)

                # Saving the binary mask as .npy and image
                bev_binary_filename = f'{bev_filename}_binary'

                npy_outdir = os.path.join(args.outdir, 'bev_binary_as_npy')
                os.makedirs(npy_outdir, exist_ok=True)
                saving_path = os.path.join(npy_outdir, f"{bev_binary_filename}.npy")
                np.save(saving_path, bev_image_binary)

                # === 2. SAVE USING OPENCV (for visualization) ===
                # Convert float32 [0,1] -> uint8 [0,255]
                bev_binary_image_uint8 = (bev_image_binary * 255).astype(np.uint8)
                cv2.imwrite(os.path.join(args.outdir, output_base, 
                                        f"{bev_binary_filename}.png"), 
                                        bev_binary_image_uint8)
                
                # Apply inpainting to reduce depth noise
                bev_image_binary_inpainted = self.bev_binary_inpainting(bev_image_binary)
                bev_image_binary_inpainted_uint8 = (bev_image_binary_inpainted * 255).astype(np.uint8)
                cv2.imwrite(os.path.join(args.outdir, output_base, 
                                        f"{bev_binary_filename}_inpainting.png"), 
                                        bev_image_binary_inpainted_uint8)
        else:
            # Concatenate original image and BEV image side by side for comparison
            # This is wrong, resizing must preserve the aspect radio
            bev_resized = cv2.resize(bev_image_vis, (ci_width, ci_height))
            
            combined = np.hstack((color_image, bev_resized))
            combined_output_path = os.path.join(args.outdir, output_base, f"{output_base}_bev_comparison.png")
            if args.write_singles:
                cv2.imwrite(combined_output_path, combined)
        return bev_image_vis, bev_binary_image_uint8, bev_filename, bev_image_binary_inpainted_uint8

    def bev_pixels_to_meters(self, pixel_coords, x_min, z_min, cell_size, height, save_img_path=False):
        """
        Convert BEV pixel coordinates back to (x, z) in meters.
        
        Parameters
        ----------
        pixel_coords : array-like of shape (N, 2)
            List or array of (u, row) pixel coordinates in BEV image.
            u = column index (x direction), row = row index (y direction in image)
        x_min : float
            Minimum x value used to create the BEV (same as in pcl_to_BEV).
        z_min : float
            Minimum z value used to create the BEV (same as in pcl_to_BEV).
        cell_size : float
            Size of one pixel in meters (same as in pcl_to_BEV).
        height : int
            Height of the BEV image.
        
        Returns
        -------
        np.ndarray of shape (N, 2)
            Each row is [x, z] in meters.
        """
        
        pixel_coords = np.asarray(pixel_coords)
        u = pixel_coords[:, 0]
        row = pixel_coords[:, 1]

        # invert the image coordinate system
        v = height - 1 - row # no need to include height since origin is below for img
        #height - 1 - v = row # actually 1 + v = row
        # map back to meters
        x = u * cell_size + x_min
        z = v * cell_size + z_min
        print(f"[BEV img properties] x_min: {x_min} (m), z_min: {z_min} (m), cell_size: {cell_size} px/m, BEV img height {height} pixels")
        
        # Optional visualization
        if save_img_path:
            # Create a blank image
            # Determine the image size in pixels (scale meters to pixels)
            margin = 50  # pixels around path
            scale = 100  # pixels per meter for visualization

            x_px = ((x - x.min()) * scale).astype(np.int32) + margin
            z_px = ((z - z.min()) * scale).astype(np.int32) + margin

            img_height = z_px.max() + margin
            img_width = x_px.max() + margin
            img = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255  # white background

            # Draw path
            for k in range(1, len(x_px)):
                cv2.line(img, (x_px[k-1], z_px[k-1]), (x_px[k], z_px[k]), (255, 0, 0), 2)  # blue line
                cv2.circle(img, (x_px[k], z_px[k]), 3, (0, 0, 255), -1)  # red points

            # Draw grid lines every 0.5 meters
            grid_spacing = int(0.5 * scale)
            for gx in range(0, img_width, grid_spacing):
                cv2.line(img, (gx, 0), (gx, img_height), (200, 200, 200), 1)
            for gz in range(0, img_height, grid_spacing):
                cv2.line(img, (0, gz), (img_width, gz), (200, 200, 200), 1)
            save_path = '/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/output_dir/astart_planner_part3/path_in_meters.png'
            cv2.imwrite(save_path, img)
        return np.stack((x, z), axis=1)
    

    def world_to_bev_coords(self, point: np.ndarray, x_min: float, z_min: float, cell_size: float, height: int) -> tuple[int, int]:
        """
        Convert a 3D world point (x, y, z) to BEV image pixel coordinates.
        """
        u = int((point[0] - x_min) / cell_size)
        v = int((point[2] - z_min) / cell_size)
        v_img = height - v - 1  # invert Y for image coordinates
        return u, v_img

    def get_nearest_point_bev_pixel(self, pcd: o3d.geometry.PointCloud, queries: np.ndarray, cell_size: float = 0.01) -> tuple[tuple[int, int], tuple[int, int]]:
        """
        Given a point cloud and a query point, return:
        1) BEV pixel coordinates of the nearest point
        2) Simplified nearest point tuple for easy storage/use

        Returns: List with (u_nearest, v_nearest) for each query
        """
        nearest_bev_pixel = []
        points = np.asarray(pcd.points)

        # --- KDTree search for nearest point ---
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        for query in queries:
            if query is None:
                nearest_bev_pixel.append(None)
            else:
                _, idx, _ = pcd_tree.search_knn_vector_3d(query, 1)
                nearest_point = points[idx[0]]

                # --- Compute BEV bounds ---
                x_min, x_max = points[:, 0].min(), points[:, 0].max()
                z_min, z_max = points[:, 2].min(), points[:, 2].max()
                width = int((x_max - x_min) / cell_size)
                height = int((z_max - z_min) / cell_size)

                # --- Map query and nearest point to BEV pixels ---
                u_query, v_query = self.world_to_bev_coords(query, x_min, z_min, cell_size, height)
                u_nearest, v_nearest = self.world_to_bev_coords(nearest_point, x_min, z_min, cell_size, height)
                nearest_bev_pixel.append((u_nearest, v_nearest))

        return nearest_bev_pixel

class Sam2Wrapper():
    def __init__(self, device, sam2_checkpoint, model_cfg):       
        self.predictor = self.get_sam2_model_predictor(device, sam2_checkpoint, model_cfg)

    def show_mask(self, mask, image, random_color=False, borders=True):
        """Overlay a mask on a BGR image."""
        if random_color:
            color = np.random.random(3) * 255
        else:
            color = np.array([255, 144, 30])  # BGR (blue tone)
        overlay = image.copy()

        # Ensure mask is binary
        mask = (mask > 0).astype(np.uint8)

        # Apply overlay with alpha blending
        overlay[mask == 1] = (0.4 * overlay[mask == 1] + 0.6 * color).astype(np.uint8)

        # Optional borders
        if borders:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
            cv2.drawContours(overlay, contours, -1, (255, 255, 255), thickness=2)

        return overlay


    def show_points(self, image, coords, labels):
        """Draw labeled points on a BGR image."""
        overlay = image.copy()
        for (x, y), label in zip(coords, labels):
            color = (0, 255, 0) if label == 1 else (0, 0, 255)
            cv2.drawMarker(overlay, (int(x), int(y)), color,
                        markerType=cv2.MARKER_STAR, markerSize=15,
                        thickness=2, line_type=cv2.LINE_AA)
        return overlay


    def show_box(self, image, box):
        """Draw a bounding box on a BGR image."""
        overlay = image.copy()
        x0, y0, x1, y1 = map(int, box)
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 255, 0), thickness=2)
        return overlay


    def show_masks(self, image_filename, image, masks, scores, point_coords=None, box_coords=None,
                input_labels=None, borders=True, save_dir="output_masks", write_singles=False):
        """Generate and save mask visualizations using OpenCV only."""
        os.makedirs(save_dir, exist_ok=True)

        # Convert RGB → BGR (since you load with Pillow)
        image_bgr = image #cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        for i, (mask, score) in enumerate(zip(masks, scores)):
            img_vis = image_bgr.copy()

            # Overlay mask
            img_vis = self.show_mask(mask, img_vis, borders=borders)

            # Add points if provided
            if point_coords is not None and input_labels is not None:
                img_vis = self.show_points(img_vis, point_coords, input_labels)

            # Add box if provided
            if box_coords is not None:
                img_vis = self.show_box(img_vis, box_coords)

            # Optional text
            if len(scores) > 1:
                text = f"Mask {i+1}, Score: {score:.3f}"
                cv2.putText(img_vis, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (255, 255, 255), 2, cv2.LINE_AA)

            # Save file
            filename = f"{image_filename}_sam2_mask_{i+1}_score_{score:.3f}.png"
            filepath = os.path.join(save_dir, filename)
            if write_singles:
                cv2.imwrite(filepath, img_vis)
                #print(f"Saved: {filepath}")
            return img_vis, filename

    def infer(self, sam2_model, image_filename, image_input, device, display_masks=False, save_dir='output_masks', write_singles=False):
        predictor = sam2_model
        image = image_input.copy()
        predictor.set_image(image)
        
        height, width, channels =  image.shape
        input_point = np.array([[width/2, height*0.95]]) # prompt the center, bottom pixel (closer to the camera frame)
        input_label = np.array([1]) # 1 for foreground

        masks, scores, logits = predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=False,
        )

        # Propagate the mask for future iterations
        """ mask_input = logits[np.argmax(scores), :, :]  # Choose the model's best mask
        masks, scores, _ = predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            mask_input=mask_input[None, :, :],
            multimask_output=False,
        ) """
        if display_masks:
            img_masked, filename_img_masked = self.show_masks(image_filename, image, masks, scores, point_coords=input_point, input_labels=input_label, borders=False, save_dir=save_dir, write_singles=write_singles)
        return masks, img_masked, filename_img_masked

    def get_sam2_model_predictor(self, device, sam2_checkpoint, model_cfg):
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=device)

        predictor = SAM2ImagePredictor(sam2_model)
        return predictor

from dataclasses import dataclass
from typing import Optional

@dataclass
class PipelineArgs:
    encoder: str
    load_from: str
    max_depth: float
    img_path: str
    outdir: str
    focal_length_x: float
    focal_length_y: float
    pred_only: bool
    save_pcl: bool
    plot_summary: bool
    grayscale: bool
    write_singles: bool
    sam2_checkpoint: Optional[str]
    sam2_model_cfg: Optional[str]

class DepthPipeline():
    def __init__(self, args: PipelineArgs):
        self.args = args
        self.device = self.get_device()
        self.sam2_checkpoint = args.sam2_checkpoint
        self.model_cfg = args.sam2_model_cfg
        print('loading DA2 and SAM2 models...')
        self.depth_model = DepthModel(args)
        self.sam2_wrapper = Sam2Wrapper(device=self.device, sam2_checkpoint=self.sam2_checkpoint, model_cfg=self.model_cfg)


    def get_device(self):
        if torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")

        print(f"using device: {device}")

        if device.type == "cuda":
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        return device
    
    def process_img(self, img_path):
        raw_image0 = cv2.imread(img_path)
        raw_image = cv2.resize(raw_image0, (720, 480))
        raw_image_c = raw_image.copy()
        image_filename = os.path.splitext(os.path.basename(img_path))[0]
        
        masks, img_sam2_masked, filename_img_masked = self.sam2_wrapper.infer(self.sam2_wrapper.predictor, 
                                                                            image_filename, 
                                                                            raw_image, 
                                                                            device=self.device, 
                                                                            display_masks=True, 
                                                                            save_dir=os.path.join(self.args.outdir, image_filename), 
                                                                            write_singles=self.args.write_singles)

        binary_mask = (masks[0] > 0).astype(np.uint8)
        depth_pred, bev_image_vis, bev_binary_image_uint8, bev_filename, bev_image_binary_inpainted_uint8, depth_img, depth_filename, pcd = self.depth_model.get_depth(
            filename=img_path, 
            image_input=raw_image, 
            mask=binary_mask, 
            args=self.args)
        
        to_3ch = lambda img : cv2.merge([img, img, img]) # shape (H, W, 3)
        bev_inpainted_3ch = to_3ch(bev_image_binary_inpainted_uint8)
        bev_binary_3ch = to_3ch(bev_binary_image_uint8)
        

        if self.args.plot_summary:
            self.plot_img_summary([raw_image_c, img_sam2_masked, depth_img, bev_image_vis, bev_binary_3ch, bev_inpainted_3ch], 
                                os.path.join(self.args.outdir, 
                                            image_filename, 
                                            image_filename + '_summary.png'))
        return pcd, bev_image_vis, bev_binary_image_uint8, bev_image_binary_inpainted_uint8

    def plot_img_summary(self, images, output_path, cell_w=400, cell_h=300, spacing=20):
        num_images = len(images)
        import math
        # Automatically compute grid size (rows x cols)
        cols = math.ceil(math.sqrt(num_images))        # number of columns
        rows = math.ceil(num_images / cols)            # number of rows
        
        resized_images = []
        for img in images:
            h, w = img.shape[:2]
            scale = min(cell_w / w, cell_h / h)  # keep aspect ratio
            new_w, new_h = int(w * scale), int(h * scale)
            resized = cv2.resize(img, (new_w, new_h))
            
            # create a white cell and center the image
            cell = np.ones((cell_h, cell_w, 3), dtype=np.uint8) * 255
            y_off = (cell_h - new_h) // 2
            x_off = (cell_w - new_w) // 2
            cell[y_off:y_off + new_h, x_off:x_off + new_w] = resized
            resized_images.append(cell)
        
        # Compute final canvas size
        canvas_h = rows * cell_h + (rows + 1) * spacing
        canvas_w = cols * cell_w + (cols + 1) * spacing
        canvas = np.ones((canvas_h, canvas_w, 3), dtype=np.uint8) * 255
        
        # Paste cells onto canvas
        idx = 0
        for r in range(rows):
            for c in range(cols):
                if idx >= num_images:
                    break
                y0 = spacing + r * (cell_h + spacing)
                x0 = spacing + c * (cell_w + spacing)
                canvas[y0:y0 + cell_h, x0:x0 + cell_w] = resized_images[idx]
                idx += 1
        
        # Save the final summary image
        cv2.imwrite(output_path, canvas)


def main():

    parser = argparse.ArgumentParser(description='Generate depth maps and point clouds from images.')
    parser.add_argument('--encoder', default='vitl', type=str, choices=['vits', 'vitb', 'vitl', 'vitg'])
    parser.add_argument('--load-from', required=True, type=str, help='Path to depth model weights')
    parser.add_argument('--max-depth', default=20.0, type=float, help='Maximum predicted depth ')
    parser.add_argument('--img-path', required=True, type=str, help='Image file, directory, or glob pattern for inference')
    parser.add_argument('--outdir', default='./vis_pointcloud', type=str, help='Directory to save inference results')
    parser.add_argument('--focal-length-x', default=1580.29, type=float, help='Camera calibration parameter')
    parser.add_argument('--focal-length-y', default=1581.78, type=float, help='Camera calibration parameter')
    parser.add_argument('--pred-only', action='store_true', help='only display the depth prediction')
    parser.add_argument('--save-pcl', action='store_true', help='save pointcloud to a .ply')
    parser.add_argument('--plot-summary', action='store_true', help='save a summary img of depth, sam2 mask, and BEV')
    parser.add_argument('--grayscale', action='store_true', help='do not apply colorful palette to depth')
    parser.add_argument('--write-singles', action='store_true', help='write images for each module')
    parser.add_argument('--sam2-checkpoint', default=None, type=str, required=True,)
    parser.add_argument('--sam2-model-cfg', default=None, type=str, required=True,)
    args = parser.parse_args()
    
    pipeline_args = PipelineArgs(
        encoder=args.encoder,
        load_from=args.load_from,
        max_depth=args.max_depth,
        img_path=args.img_path,
        outdir=args.outdir,
        focal_length_x=args.focal_length_x,
        focal_length_y=args.focal_length_y,
        pred_only=args.pred_only,
        save_pcl=args.save_pcl,
        plot_summary=args.plot_summary,
        grayscale=args.grayscale,
        write_singles=args.write_singles,
        sam2_checkpoint=args.sam2_checkpoint,
        sam2_model_cfg=args.sam2_model_cfg,
    )

    if os.path.isfile(args.img_path):
        if args.img_path.endswith('txt'):
            with open(args.img_path, 'r') as f:
                filenames = f.read().splitlines()
        else:
            filenames = [args.img_path]
    else:
        filenames = glob.glob(os.path.join(args.img_path, '**/*'), recursive=True)
    
    
    depth_pipeline = DepthPipeline(pipeline_args)

    for k, img_path in enumerate(filenames):
        img_file_name = os.path.splitext(os.path.basename(img_path))[0]
        print(f'Progress {k+1}/{len(filenames)}: {img_file_name}')

        pcd, bev_image_vis, bev_binary_image_uint8, bev_image_binary_inpainted_uint8 = depth_pipeline.process_img(img_path)

if __name__ == '__main__':
    main()

    """ python3 DepthPipeline.py \
        --encoder "vitl"  \
        --load-from "/home/steven/Documents/phd_while_alienware/repos (personal)/DINOv3/monocular_depth_estimation/Depth-Anything-V2/checkpoints/metric_depth/depth_anything_v2_metric_hypersim_vitl.pth"  \
        --sam2-checkpoint "/home/steven/Documents/phd_while_alienware/repos (personal)/DINOv3/SAM2_rivers/sam2/checkpoints/sam2.1_hiera_large.pt" \
        --sam2-model-cfg "configs/sam2.1/sam2.1_hiera_l.yaml" \
        --img-path "/home/steven/Documents/phd_while_alienware/repos (personal)/DINOv3/BEV/mask_to_point_cloud/output/frames_output/frame_23.png" \
        --outdir "/home/steven/Documents/phd_while_alienware/repos (personal)/DINOv3/monocular_depth_estimation/Depth-Anything-V2/vis_moloplanner/clean_test" \
        --save-pcl \
        --pred-only
    """