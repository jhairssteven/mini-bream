import os
import numpy as np
from moloplanner.DepthPipeline import DepthPipeline, PipelineArgs
import argparse, yaml
import open3d as o3d

# image planner depends
from geotiff_global_planner.scripts.geo_transform_utils import GeoImageTransformer
from geotiff_global_planner.scripts.image_planner import GridPlanner, draw_path_on_img, AStartPlanner
import os

IMG_FILE = '/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/input_imgs/frame_2.png'

class Moloplanner():
    
    def __init__(self):
        self.pipeline_args, self.config = self.parse_args()
        self.depth_pipeline = DepthPipeline(self.pipeline_args)

    def get_running_args(self):
        return self.pipeline_args, self.config
    
    def run(self, astart_outdir, pcd_bev_binary_mask, filebasename, pcd=None):
        depth_pipeline = self.depth_pipeline
        if pcd is None:
            pcd, bev_image_vis, bev_binary_image_uint8, bev_image_binary_inpainted_uint8 = depth_pipeline.process_img(IMG_FILE)

        start_point = np.array([0.0, 0.0, 0.0])
        goal_point = np.array([10, 4, 5]) # x, y, z # given by the global planner

        # the pixel coordinates of BEV projection of given points in the PCD
        BEV_start, BEV_goal = depth_pipeline.depth_model.get_nearest_point_bev_pixel(pcd, [start_point, goal_point])

        astart_planner = AStartPlanner()
        path = astart_planner.plan(image_path=None, 
                            image_array=pcd_bev_binary_mask, 
                            start=np.array([BEV_start[1], BEV_start[0]]),
                            goal=None, #np.array([BEV_goal[1], BEV_goal[0]]),
                            save_output=True, 
                            output_dir=astart_outdir,
                            filename= filebasename + '_astart_planner')

    def parse_args(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--config', type=str, required=True, help='YAML configuration file')
        parser.add_argument('--override', nargs='*', help='Optional overrides like key=value')
        args = parser.parse_args()

        # Load YAML file
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)

        # Optional overrides (e.g. --override pipeline.max-depth=15.0)
        if args.override:
            for kv in args.override:
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

if __name__ == '__main__':
    import cv2, glob
    molo_planner = Moloplanner()
    pipeline_args, config = molo_planner.get_running_args()
    
    
    astart_planner_args = config['astart_planner']
    moloplanner_args = config['moloplanner']
    
    pcds_filenames = glob.glob(os.path.join(moloplanner_args['pcds_path'], '**/*'), recursive=True)
    bev_npys_filenames = glob.glob(os.path.join(moloplanner_args['bevs_npy'], '**/*'), recursive=True)
    
    print(f'Files to process: pcds: {len(pcds_filenames)}, bev_npys {len(bev_npys_filenames)}')
    for idx, pcd_path in enumerate(pcds_filenames):
        original_img_filename = os.path.splitext(os.path.basename(pcd_path))[0]
        print(f'Progress {idx+1}/{len(pcds_filenames)}: {original_img_filename}')

        bev_npy = os.path.join(moloplanner_args['bevs_npy'], original_img_filename + '_bev_binary.npy')
        
        
        print(f'Reading pcl: {os.path.basename(pcd_path)}')
        print(f'Reading bev_npy: {os.path.basename(bev_npy)}')
        pcd = o3d.io.read_point_cloud(pcd_path)
        pcd_bev_binary_mask = np.load(bev_npy)
        
        # inpainting
        # Normalize for mask operations (non-zero = valid)
        bev_binary_bool = (pcd_bev_binary_mask > 0).astype(np.uint8)

        # Apply morphological closing to fill gaps
        kernel = np.ones((10, 1), np.uint8)   # increase size for thicker fill
        bev_binary_inpainted = cv2.morphologyEx(bev_binary_bool, cv2.MORPH_CLOSE, kernel)
        
        
        molo_planner.run(
            astart_planner_args['outdir'], 
            bev_binary_inpainted, 
            filebasename=original_img_filename, 
            pcd=pcd)
    
    
    """ 
     Usage:
     python3 -m moloplanner.moloplanner --config config.yaml --override depth_pipeline.max_depth=15 
    """
    

