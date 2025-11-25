import os
import numpy as np
from mission_planner.moloplanner.DepthPipeline import DepthPipeline, PipelineArgs
import argparse, yaml
import open3d as o3d

# image planner depends
from mission_planner.geotiff_global_planner.scripts.image_planner import AStartPlanner
import mission_planner.moloplanner.planning_utils as planning_utils
import cv2

class Moloplanner():
    
    def __init__(self, config_path=None, overrides=None):
        self.config_path = config_path
        self.overrides = overrides or {}

        self.pipeline_args, self.config = self.parse_args(config_path, overrides)
        self.depth_pipeline = DepthPipeline(self.pipeline_args)
        self.astart_planner_args = self.config['astart_planner']
        self.moloplanner_args = self.config['moloplanner']

    def get_running_args(self):
        return self.pipeline_args, self.config
    
    def bev_pixel_astart_path(self, 
            start_point=np.array([0.0, 0.0, 0.0]), goal_point=None, 
            pcd_bev_binary_mask=None, img_id : str = 'unnamed', input_img_array : np.ndarray = None, pcd=None):
        """ 
            Given a pointcloud 'pcd' and a (start, goal) pair of points, return a A* path in pixel coordinates
            in the 'pcd_bev_binary_mask' or 'input_img_array' (which ever is given).

            Args:
            
            Start and goal point coordinates in the 'pcd' for the A* path generation
            start_point: Numpy array (x, y, z). Defaults to the origin point in the 'pcd'
            goal_point: Numpy array (x, y, z)
            pcd: 3D pointcloud

            Returns:
            path (ndarray) Format: [[i0, j0], [i1, j1], ...]: The pixel coordinates of the path in 'image_path'.
            
        """
        if input_img_array is not None:
            if not (pcd_bev_binary_mask is None and pcd is None):
                raise ValueError("'img_filepath' is defined, 'pcd' and its bev binary should not be defined in this case")
        
        depth_pipeline = self.depth_pipeline
        if pcd is None:
            pcd, bev_image_vis, bev_binary_image_uint8, bev_image_binary_inpainted_uint8 = depth_pipeline.process_img(img_id, input_img_array)

        # the pixel coordinates of BEV projection of given points in the PCD
        BEV_start, BEV_goal = depth_pipeline.depth_model.get_nearest_point_bev_pixel(pcd, [start_point, goal_point])

        astart_planner = AStartPlanner()
        path = astart_planner.plan(image_path=None, 
                            image_array=bev_image_binary_inpainted_uint8, 
                            start=np.array([BEV_start[1], BEV_start[0]]) if BEV_start is not None else None,
                            goal=np.array([BEV_goal[1], BEV_goal[0]]) if BEV_goal is not None else None,
                            save_output=self.astart_planner_args['save_output'], 
                            output_dir=self.astart_planner_args['outdir'],
                            filename =img_id + '_astart_planner')
        return path, pcd

    def tf_next_waypoint_to_pcl_frame(self, next_waypoint_gps, camera_frame_origin_gps, boat_heading_deg):
        """ 
        Args:
            next_waypoint_gps (tuple[float, float]):
                Target waypoint as (latitude, longitude) in decimal degrees.
            camera_frame_origin_gps (tuple[float, float]):
                GPS coordinates (latitude, longitude) of the camera frame origin in decimal degrees.
            camera_frame_orientation_deg (float):
                Orientation of the camera frame (degrees) w.r.t to x axis of UTM origin. This is used as the rotation
                applied when converting GPS coordinates into the local frame.

         Returns: goal point (x, y, z) numpy arrays """
        
        camera_frame_orientation_deg = boat_heading_deg - 90 # (camera's x axis is 90deg CW rotated w.r.t the boat's heading.)
        
        nw_in_camera_frame = planning_utils.gps_to_local_frame(
            target_gps=next_waypoint_gps,
            origin_gps=camera_frame_origin_gps,
            frame_orientation_deg=camera_frame_orientation_deg
        )
        x, y = nw_in_camera_frame
        goal_point = np.array([x, 0, y]) # camera's y is set to z coordinate to match coordinate frame in pcl.
        
        return goal_point

    def tf_pcl_coordinates_to_gps(self, pcl_xz_coordinates, camera_frame_origin_gps, boat_heading_deg):
        """ 
        Args:
            pcl_xz_coordinates: np.ndarray, shape (N, 2): Each row is [x, z] (m) coordinates in PCL frame
        Returns
        -------
        tuple : (lat, lon)
            GPS array coordinates corresponding to '*pcl_xz_coordinates*'.
        """

        if not (pcl_xz_coordinates.ndim == 2 and pcl_xz_coordinates.shape[1] == 2):
            raise ValueError(f"Expected 'pcl_xz_coordinates' of shape (N, 2), got {pcl_xz_coordinates.shape}")
        gps_path = []
        for pcl_xz_coordinate in pcl_xz_coordinates:            
            delta_local = np.array([pcl_xz_coordinate[0], pcl_xz_coordinate[1]])

            camera_frame_orientation_deg = boat_heading_deg - 90 # (camera's x axis is 90deg CW rotated w.r.t the boat's heading.)
            
            equivalent_gps = planning_utils.local_frame_to_gps(
                delta_local, 
                origin_gps=camera_frame_origin_gps, 
                frame_orientation_deg=camera_frame_orientation_deg)
        
            gps_path.append(equivalent_gps)
        return gps_path
    
    def load_config(self, config_path, overrides=None):
        """Load YAML config file and apply optional key=value overrides."""

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Optional overrides (e.g. --override pipeline.max-depth=15.0)
        if overrides:
            for kv in overrides:
                key, value = kv.split('=')
                section, param = key.split('.')
                # Basic auto-casting to number/bool when possible
                if value.lower() in ('true', 'false'):
                    value = value.lower() == 'true'
                elif value.replace('.', '', 1).isdigit():
                    value = float(value) if '.' in value else int(value)
                config[section][param] = value

        # Extract parameter groups
        pipeline_args = PipelineArgs(**config['depth_pipeline'])

        return pipeline_args, config
    
    def parse_args(self, config_path=None, overrides=None):
        if config_path:
            return self.load_config(config_path, overrides)
        
        # Parse as CLI arguments
        parser = argparse.ArgumentParser()
        parser.add_argument('--config', type=str, required=True, help='YAML configuration file')
        parser.add_argument('--override', nargs='*', help='Optional overrides like key=value')
        args = parser.parse_args()

        return self.load_config(args.config, args.override)
    
    def create_grid(self, size=10, step=1):
        lines = []
        points = []
        
        # Create parallel lines along X and Y
        for i in np.arange(-size, size+step, step):
            # Lines along X (parallel to X axis)
            points.append([i, 0, -size])
            points.append([i, 0, size])
            lines.append([len(points)-2, len(points)-1])
            
            # Lines along Z (parallel to Z axis)
            points.append([-size, 0, i])
            points.append([size, 0, i])
            lines.append([len(points)-2, len(points)-1])
                
        # Create LineSet
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set.colors = o3d.utility.Vector3dVector([[0.7, 0.7, 0.7] for _ in lines])
        return line_set

    def plot_path_on_pointcloud(self, coords_m, pcd=None, point_size=2.0):
        """
        Visualize a path (coords_m) on top of a point cloud using Open3D.
        
        Parameters
        ----------
        coords_m : np.ndarray of shape (N,2)
            X-Z coordinates of the path in meters.
        pcd : open3d.geometry.PointCloud or None
            Original point cloud to show in the background. If None, shows only path.
        point_size : float
            Size of points in the visualizer.
        """

        vis_objects = []

        # Add point cloud if provided
        if pcd is not None:
            vis_objects.append(pcd)

        # Convert path coords to 3D (y = 0)
        path_points = np.zeros((coords_m.shape[0], 3))
        path_points[:, 0] = coords_m[:, 0]  # x
        path_points[:, 2] = coords_m[:, 1]  # z
        
        # Create LineSet for the path
        path_lines = [[i, i+1] for i in range(len(path_points)-1)]
        colors = [[1.0, 0.0, 0.0] for _ in path_lines]  # red

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(path_points),
            lines=o3d.utility.Vector2iVector(path_lines)
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)

        # Optionally, draw points as small spheres along the path
        path_spheres = o3d.geometry.PointCloud()
        path_spheres.points = o3d.utility.Vector3dVector(path_points)
        path_spheres.colors = o3d.utility.Vector3dVector(np.tile([1,0,0], (len(path_points),1)))

        vis_objects.extend([line_set, path_spheres])

        # 2. Create a coordinate frame
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0,
            origin=[0, 0, 0]
        )
        grid = self.create_grid(size=10, step=1)
        vis_objects.extend([grid, axis])

        # Visualize
        o3d.visualization.draw_geometries(vis_objects, point_show_normal=False)
    
    def get_gps_local_astart_path(self, img_id: str, input_img_array: np.ndarray, next_waypoint_gps, camera_frame_origin_gps, boat_heading_deg):
        """ Given and input image and the pose of the camera's frame w.r.t to world GPS coordinates, return
         a obstacle-aware A* path from the BEV representation of the image. 
        Args:
            img_id (str): String identifying the input image 
        """

        goal_point = self.tf_next_waypoint_to_pcl_frame(
            next_waypoint_gps=next_waypoint_gps, 
            camera_frame_origin_gps=camera_frame_origin_gps, 
            boat_heading_deg=boat_heading_deg)
        
        bev_pixel_astart_path, pcd = self.bev_pixel_astart_path(
            start_point=np.array([0.0, 0.0, 0.0]), goal_point=goal_point, 
            pcd_bev_binary_mask=None, 
            img_id=img_id,
            input_img_array=input_img_array,
            pcd=None)
        
        # Swapt path ([[i, j], ...]  (row, col)) to (col, row)
        astart_path = bev_pixel_astart_path[:, [1, 0]]


        pcl_xz_coordinates = self.depth_pipeline.depth_model.bev_pixels_to_meters(
            astart_path,
            self.depth_pipeline.depth_model.x_min, 
            self.depth_pipeline.depth_model.z_min, 
            self.depth_pipeline.depth_model.cell_size, 
            self.depth_pipeline.depth_model.height,
            save_img_path=True,
            output_dir=self.moloplanner_args['outdir'],
            img_id=img_id)
        # 3D visualize the pcd and the path
        #self.plot_path_on_pointcloud(pcl_xz_coordinates, pcd=pcd)
        gps_local_astart_path = self.tf_pcl_coordinates_to_gps(pcl_xz_coordinates, camera_frame_origin_gps, boat_heading_deg)
        
        # Subsample path since the GPS resolution is less than 1m.
        #gps_local_astart_path = gps_local_astart_path[::50]
        return gps_local_astart_path
        
if __name__ == '__main__':
    molo_planner = Moloplanner()
    pipeline_args, config = molo_planner.get_running_args()
    
    astart_planner_args = config['astart_planner']
    moloplanner_args = config['moloplanner']
    

    # === params
    next_waypoint_gps=(40.443026, -86.763256)
    camera_frame_origin_gps=(40.44286291645092, -86.76329222468132)
    boat_heading_deg=90
    
    img_filepath = moloplanner_args['test_img']
    
    img_id = os.path.splitext(os.path.basename(img_filepath))[0]
    input_img_array = cv2.imread(img_filepath)

    gps_local_astart_path = molo_planner.get_gps_local_astart_path(img_id, input_img_array,
                                                                   next_waypoint_gps,
                                                                   camera_frame_origin_gps,
                                                                   boat_heading_deg)
    """ 
     Usage:
     python3 -m moloplanner.moloplanner --config config.yaml --override depth_pipeline.max_depth=15 
    """
    

