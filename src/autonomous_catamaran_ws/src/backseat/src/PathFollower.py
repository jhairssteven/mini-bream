import numpy as np
import pandas as pd
import rospy
from matplotlib import pyplot as plt
import copy
import time
import dubins
#import reeds_shepp
import pathlib
import os
from NavigationTools import *
#from CsvLogger import *

'''
class ReedsSheppPath:
    def __init__(self, mission, turn_radius=rospy.get_param("dubins_radius"),
                 step_size=rospy.get_param("dubins_step_size"), log=True,):
        self.turning_radius = turn_radius    # Meters
        self.step_size = step_size           # Meters
        rospy.logdebug("Reeds-Shepp    radius %f, step %f", self.turning_radius, self.step_size)
        self.gps_calc = NavigationTools.GpsCalculations()
        self.path = []
        path = []
        main_wp_idxs = [0]
        for idx,wp in enumerate(mission):
            x = wp.ToUTM().pose.utm_x
            y = wp.ToUTM().pose.utm_y
            angle = wp.pose.head            # ENU Frame angle (East = 0 deg, North = 90 deg)
            q1 = (x, y, angle)
            if idx > 0:
                if self.__get_dist(q0, q1) > 2*self.step_size:
                    configurations = reeds_shepp.path_sample(q0, q1, self.turning_radius,self.step_size)
                else:
                    configurations = [q0, q1]
                last = main_wp_idxs[-1]
                main_wp_idxs.append(last+len(configurations)+1)
                path.extend(configurations)
            q0 = q1
        cnt = 0
        for i,p in enumerate(path):
            zone = mission[0].ToUTM().pose.utm_zone
            if main_wp_idxs[cnt] != i:
                wp = NavigationTools.Waypoint(utm_x = p[0], utm_y = p[1], utm_zone = zone, head = p[2])
                self.path.append(wp)
            else:
                #print(cnt,mission[cnt])
                wp = copy.deepcopy(mission[cnt])
                wp.pose.head = wp.pose.head # ENU Frame angle (East = 0 deg, North = 90 deg)
                self.path.append(wp)
                cnt += 1
        wp = copy.deepcopy(mission[cnt])
        wp.pose.head = wp.pose.head         # ENU Frame angle (East = 0 deg, North = 90 deg)
        self.path.append(wp)
        if (log == True):
            self.__log_path()
    
    def __get_dist(self,q0,q1):
        q0 = np.array(q0)
        q1 = np.array(q1)
        return np.linalg.norm(q0[0:-1]-q1[0:-1])
        
    def __log_path(self):
        tmp_list = []
        for p in self.path:
            tmp_list.append({'x':p.pose.utm_x,'y':p.pose.utm_y,'head':p.pose.head})
        df = pd.DataFrame(tmp_list)
        cur_file_path = str(pathlib.Path(__file__).parent.resolve())
        filename = time.strftime("%Y%m%d-%H%M%S")
        filename += '.csv'
        log_dir = cur_file_path + '/log/planner/'
        if not os.path.exists(log_dir):
                os.makedirs(log_dir)
        filename = log_dir + filename 
        print('Plan logged: %s'%(filename))
        df.to_csv(r'%s'%(filename), index = False, header=True)

    def plot(self):
        x = [wp.pose.utm_x for wp in self.path]
        y = [wp.pose.utm_y for wp in self.path]
        plt.plot(x,y)
        tri_start = self.get_car(q=[x[0],y[0],self.path[0].pose.head])
        plt.plot(tri_start[:,0], tri_start[:,1], 'g-',label='start')
        tri_end = self.get_car(q=[x[-1],y[-1],self.path[-1].pose.head])
        plt.plot(tri_end[:,0], tri_end[:,1], 'r-',label='end')
        plt.legend()
        plt.axis('equal')
        plt.show()
    
    def get_point(self, center, radius, orin):
        x = center[0] + radius * np.cos(orin)
        y = center[1] + radius * np.sin(orin)
        return (x,y)

    def get_car(self,q):
        a = self.get_point(q[:-1], self.step_size, q[2])
        b = self.get_point(q[:-1], self.step_size/2, q[2]+150./180.*np.pi)
        c = self.get_point(q[:-1], self.step_size/2, q[2]-150./180.*np.pi)
        tri = np.array([a,b,c,a])
        return tri

    def __str__(self):
        print('Path:')
        print('{0:13s}{1:14s}{2:10s}'.format('x','y','angle'))
        for wp in self.path:
            print('{0:10.4f}, {1:10.4f}, {2:6.4f}'.format(wp.pose.utm_x, wp.pose.utm_y, wp.pose.head))
        print(']')
        return ''

    def __len__(self):
        return len(self.path)
    
    def __getitem__(self, idx):
        return self.path[idx]
'''
    
class DubinsPath:
    def __init__(self, mission, turn_radius=rospy.get_param("dubins_radius"),
                 step_size=rospy.get_param("dubins_step_size"), log=True,):
        # print('0'*20)
        # for wp in mission:
        #   print(wp)
        # print('0'*20)
        self.turning_radius = turn_radius    # Meters
        self.step_size = step_size           # Meters
        rospy.logdebug("Dubins    radius %f, step %f", self.turning_radius, self.step_size)
        self.gps_calc = NavigationTools.GpsCalculations()
        self.path = []
        path = []
        main_wp_idxs = [0]
        for idx,wp in enumerate(mission):
            x = wp.ToUTM().pose.utm_x
            y = wp.ToUTM().pose.utm_y
            angle = wp.pose.head            # ENU Frame angle (East = 0 deg, North = 90 deg)
            q1 = (x, y, angle)
            if idx > 0:
                if self.__get_dist(q0, q1) > 2*self.step_size:
                    tmp_path = dubins.shortest_path(q0, q1, self.turning_radius)
                    configurations, _ = tmp_path.sample_many(self.step_size)
                else:
                    configurations = [q0, q1]
                last = main_wp_idxs[-1]
                main_wp_idxs.append(last+len(configurations)+1)
                path.extend(configurations)
            q0 = q1
        cnt = 0
        for i,p in enumerate(path):
            zone = mission[0].ToUTM().pose.utm_zone
            if main_wp_idxs[cnt] != i:
                wp = NavigationTools.Waypoint(utm_x = p[0], utm_y = p[1], utm_zone = zone, head = p[2])
                self.path.append(wp)
            else:
                #print(cnt,mission[cnt])
                wp = copy.deepcopy(mission[cnt])
                wp.pose.head = wp.pose.head # ENU Frame angle (East = 0 deg, North = 90 deg)
                self.path.append(wp)
                cnt += 1
        wp = copy.deepcopy(mission[cnt])
        wp.pose.head = wp.pose.head         # ENU Frame angle (East = 0 deg, North = 90 deg)
        self.path.append(wp)
        if (log == True):
            pass
            #self.__log_path()
            #self.plot()
    
    def __get_dist(self,q0,q1):
        q0 = np.array(q0)
        q1 = np.array(q1)
        return np.linalg.norm(q0[0:-1]-q1[0:-1])
        
    def __log_path(self):
        tmp_list = []
        origin_utm_x = 0*self.path[0].pose.utm_x
        origin_utm_y = 0*self.path[0].pose.utm_y
        for p in self.path:
            #tmp_list.append({'x':p.pose.utm_x-origin_utm_x,'y':p.pose.utm_y-origin_utm_y,'head':p.pose.head})
            tmp_list.append({'lat':p.pose.gps_lat,'lon':p.pose.gps_lon,'head':p.pose.head})
        df = pd.DataFrame(tmp_list)
        cur_file_path = str(pathlib.Path(__file__).parent.resolve())
        filename = time.strftime("%Y%m%d-%H%M%S")
        filename += '.csv'
        log_dir = cur_file_path + '/log/planner/'
        if not os.path.exists(log_dir):
                os.makedirs(log_dir)
        filename = log_dir + filename 
        print('Plan logged: %s'%(filename))
        df.to_csv(r'%s'%(filename), index = False, header=True)

    def plot(self):
        x = [wp.pose.utm_x for wp in self.path]
        y = [wp.pose.utm_y for wp in self.path]
        plt.scatter(x,y)
        plt.show()

    def __str__(self):
        print('Path:')
        print('{0:13s}{1:14s}{2:10s}'.format('x','y','angle'))
        for wp in self.path:
            print('{0:10.4f}, {1:10.4f}, {2:6.4f}'.format(wp.pose.utm_x, wp.pose.utm_y, wp.pose.head))
        print(']')
        return ''

    def __len__(self):
        return len(self.path)
    
    def __getitem__(self, idx):
        return self.path[idx]

class PathFollower:
    def __init__(self, mission, path_creator, log_data=True, handover_offs=rospy.get_param("handover_offs")):
        currentFolder = os.path.dirname(os.path.realpath(__file__))
        self.currentFolder = currentFolder
        if log_data:
            self.enable_logging = True
            #self.pf_logger = CsvLogger(path = currentFolder + '/log/follower', loggername = 'PathFollower', header='xte, proj_head, proj_idx, gps_lat, gps_lon, target_lat, target_lon, utm_x, utm_y, tgt_x, tgt_y',format ='%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f')
        else:
            self.enable_logging = False
        self.mission = mission
        self.path_creator = path_creator
        self.path_hdlr = path_creator(mission)
        self.original_path = self.path_hdlr.path
        print('-'*20)
        print('start: {}'.format(self.path_hdlr.path[0]))
        print('end  : {}'.format(self.path_hdlr.path[-1]))
        print('-'*20)
        self.working_path = self.original_path.copy()
        # Initialize path follower parameters
        self.mission_complete = False
        self.lap_ctr = 0
        self.current_wp = self.original_path[0]    # Empty Waypoint
        self.next_wp = self.original_path[0]       # Empty Waypoint
        # ILOS Controller Parameters and global variables
        self.work_index = 0         # Count/List index
        self.orig_index = 0         # Count/List index
        self.proj_head = 0          # Angle
        self.head = 0               # Angle
        self.ye = 0                 # meters
        self.target_speed = 1       # m/s
        self.beta_hat = 0      
        self.look_ahead_min = rospy.get_param('lookahead_min')   # Meters
        self.look_ahead_max = rospy.get_param('lookahead_max')   # Meters
        self.conv_rate = rospy.get_param('conv_rate')            # >0 constant
        self.look_ahead = rospy.get_param('lookahead')           # Meters (25m works)
        self.MIN_ALIGN_DIST = 25    # Meters
        self.MAX_DEV_FROM_PATH = rospy.get_param("replan_dist")  # Meters 300 for ILOS only
        self.NO_OF_LAPS = rospy.get_param("no_of_laps")          # Number of times to repeat the initial mission
        self.HANDOVER_OFFS = handover_offs # Meters before the end of the path to handover control of the vehicle
        
    def __measure_dist(self, wp1, wp2):
        """This function calculates the straight line distance between two points
        inputs:
            wp1: Waypoint containing the pose of the waypoint 1
            wp2: Waypoint containing the pose of the waypoint 2
        output:
            distance in meters"""
        pose_1_x, pose_1_y = wp1.pose.utm_x, wp1.pose.utm_y
        pose_2_x, pose_2_y = wp2.pose.utm_x, wp2.pose.utm_y

        return np.sqrt((pose_1_x-pose_2_x)**2+(pose_1_y-pose_2_y)**2)
    
    def dist_to_path(self, current_wp, path, current_index, rng = 10):
        """The distance_2_path function is used to get the closest point on the path (40 steps in either direction of
        known position).
        Input:
            current_wp: Current vehicle position in UTM- [norhtings, eastings, yaw]
            path: python list of all UTM coordinates along current path being traversed - [[northings, eastings, yaw]]
            current_index: the previous path UTM coordinate index that was the goal along the path - [integer]
            range: how far in both directions of the path we have to look for a potential start position - [integer in meters]
        Output:
            distance_to: The distance to closest path index point - [m]
            proj_heading: The yaw at the given closest waypoint - [rad]
            path_index: index of goal point on path - [integer]"""
        distance_to = self.__measure_dist(current_wp, path[current_index])        # calc distance to old point
        #print(hex(id(path[current_index])),hex(id(current_wp)))
        #print('wp1: {}, wp2: {}, current_index: {}'.format(current_wp,path[current_index],current_index))
        proj_index = min(len(path)-1,current_index+7)                       # Get a point ahead, without exceeding the path length
        proj_heading = path[proj_index].pose.head                           # pull out old goal yaw at old point
        path_index = current_index                                          # pull out prev index

        # for each point within 40 steps along the path (based on the step size of the dubin's path)
        for index in range(current_index - rng, current_index + rng):
            index = min(max(index, 0), len(path) - 1)
            temp_dist = self.__measure_dist(current_wp, path[index])      # calculate distance to new interated point
            #rospy.loginfo(f'c_idx: {current_index}, index: {index}, temp_dist: {temp_dist}')
            if temp_dist < distance_to:                             # if this new point is closer to the vehicle switch to it
                distance_to = temp_dist                             # save distance to new point
                proj_heading = path[proj_index].pose.head           # save goal heading at new point
                path_index = index                                  # save index of new point

        # Create distance error sign change for signed error
        # Get the rotation matrix
        R = np.transpose(np.array([[np.cos(proj_heading), - np.sin(proj_heading)], [np.sin(proj_heading), np.cos(proj_heading)]]))
        # Compute the cross-track error
        xe,ye = np.dot(R, np.array([[(current_wp.pose.utm_x - path[path_index].pose.utm_x)], [(current_wp.pose.utm_y - path[path_index].pose.utm_y)]]))
        return ye[0], proj_heading, path_index

    def ILOS(self, ye, beta_hat, proj_heading, veh_speed, sideslip = None, delta = 6, gamma = 0):
        '''
            Computes the Integral LOS guidance values to make the cross track error converge to 0,
            even in presence of external disturbances.
            Input:
                ye          : Cross track error
                beta_hat    : Integral term for the desired heading calculation
                proj_heading: Heading of the curve at the ith position.
                veh_speed   : Current vehicle speed.
                delta       : Lookahead distance.
                gamma       : Coefficient used to scale up/down the integral term beta_hat.
            Output:
                heading     : Desired heading for the vehicle
                beta_hat    : accumulated beta_hat term after the one step integration.
        '''
        if sideslip is not None:
            ##[vx, vy] = veh_speed # surge = u, sway = v ([surge_speed, sway_speed])
            ##U = np.linalg.norm(veh_speed)
            ##course_angle = np.arcsin(-vy/U)
            ##current_bearing = NavigationTools.GpsCalculations().convert_2_bearing(current_heading)*np.pi/180
            ##sideslip = course_angle - current_bearing
            ##print("sideslip: {}, cu_heading {}, current_bearing {}, course_angle {}".format(sideslip*180/np.pi, current_heading*180/np.pi, current_bearing*180/np.pi, course_angle*180/np.pi))
            # Fossen page 263 Eq.10.83
            des_course_angle = np.arctan(-ye/delta) + proj_heading
            heading = des_course_angle - sideslip
            return heading, beta_hat # same beta_hat to prevent runtime errors
        else:
            U = veh_speed   #vehicle speed
            beta_hat = beta_hat + (ye*gamma*U*delta)/(np.sqrt(delta**2+(ye+delta*beta_hat)**2))
            heading = np.arctan(-beta_hat-(ye/delta)) + proj_heading
            return heading, beta_hat
    
    def replan(self, vehicle_wp, current_wp_idx, original_path):
        '''
            This function has to be called at fixed time intervals. It implements the logic to update the control variables to move the vehicle
            based on the currect vehicle status.
            Input:
                vehicle_wp              : current position of the vehicle in the form of a waypoint.
                current_path_wp_idx     : takes the index of current target waypoint in the working path.
                original_path           : This is the path that was originally created based on the initial mission.
            Output:
                expanded_path   : path containing the new route to get on track, plus the remaining path containing the original mission
        '''
        target_idx = int(min(current_wp_idx + np.floor(self.look_ahead/self.path_hdlr.step_size), len(original_path)-np.floor(self.MIN_ALIGN_DIST/self.path_hdlr.step_size)))
        #target_idx = int(min(current_wp_idx + np.floor(self.look_ahead/self.path_hdlr.step_size), len(original_path)-1))
        '''
        The start point of the new mission is the current vehicle's position. The target position, which will be the other wp of the mission is a 
        point close, but ahead at least a minimum distance, so that there is enough space for the vehicle to get on track.
        '''
        # print('-*'*20)
        # print(vehicle_wp)
        # print('-*'*20)
        start = vehicle_wp
        end = copy.deepcopy(original_path[target_idx])
        mission = NavigationTools.Mission(waypoints=[start,end])
        recovery_path = self.path_creator(mission).path
        recovery_path.extend(copy.deepcopy(original_path[target_idx:]))
        return recovery_path

    def get_generated_paths(self):
        '''
            This function is called from the backseat controller to get access to the current paths.
            Input:
                None
            Output:
                working_path : Path - Path that is currently being used to meet the mission including any required replan.
                original_path: Path - Path that was generated initially toto meet the original requirements
        '''
        return self.working_path, self.original_path
    
    def get_look_ahead(self,ye):
        '''
            This function is used to compute a variable lookahead distance to prevent overshoot. The idea is to have a
            small lookahead when the cross-track error is large, thus, resulting in a more agressive convergence, and
            use a large lookahead when the vehicle is close to the path, thus, reducing the overshoot.
            Input:
                ye : Cross track error in meters
            Output:
                lookahead : lookahead distance in meters
        '''
        lookahead = (self.look_ahead_max-self.look_ahead_min)*np.exp(-self.conv_rate*np.abs(ye)) + self.look_ahead_min
        return lookahead

    def  update(self, current_wp, speed):
        '''
            This function has to be called at fixed time intervals. It implements the logic to update the control variables to move the vehicle
            based on the currect vehicle status.
            Input:
                current_wp  : Pass the current vehicle's position as input to compute the control actions or the required modifications to the
                existing plan.
                speed       : Current vehicle's speed.
            Output:
                None
        '''
        self.current_wp = current_wp
        # Verify the correctness of the gps data obtained
        #print('gps_lat:{}'.format(self.current_wp.pose.gps_lat))
        if(self.current_wp.pose.gps_lat != 0):
            self.ye, self.proj_head, self.work_index = self.dist_to_path(self.current_wp, self.working_path, self.work_index)
            _, _, self.orig_index = self.dist_to_path(self.current_wp, self.original_path, self.orig_index)
            #rospy.loginfo(f"orig_index, work_index: {self.orig_index}, {self.work_index}")
            
            # ======================================================================
            # Log Section
            # ======================================================================
            gps_lat,gps_lon = self.current_wp.pose.gps_lat,self.current_wp.pose.gps_lon
            utm_x,utm_y = self.current_wp.pose.utm_x,self.current_wp.pose.utm_y
            tgt_gps_lat,tgt_gps_lon = self.working_path[self.work_index].pose.gps_lat,self.working_path[self.work_index].pose.gps_lon
            tgt_x,tgt_y = self.working_path[self.work_index].pose.utm_x,self.working_path[self.work_index].pose.utm_y
            #if self.enable_logging:
                #self.pf_logger.logData((self.ye,self.proj_head, self.work_index,gps_lat,gps_lon,tgt_gps_lat,tgt_gps_lon,utm_x,utm_y,tgt_x,tgt_y))
            #rospy.logdebug('ye {} idx: {}, len_path: {}, coord: {},{}, tgt: {},{}'.format(self.ye,self.work_index,len(self.working_path),utm_x,utm_y,tgt_x,tgt_y))
            # ======================================================================
            # Verify if the end of the path has been reached
            if self.work_index >= (len(self.working_path)-self.HANDOVER_OFFS):
                # Force the cross track error to prevent any weird behavior in the controller once the misssion has been completed
                self.lap_ctr = self.lap_ctr + 1
                if self.lap_ctr >= self.NO_OF_LAPS:
                    self.mission_complete = True
                    self.ye = 0
                    self.orig_index = len(self.original_path)-1
                    self.work_index = len(self.working_path)-1
                    rospy.logdebug('Mission Completed')
                else:
                    self.orig_index = 1
                    self.work_index = 1
                    self.ye = 0
                    rospy.logdebug('Lap %d Completed'%(self.lap_ctr))
            # Replan logic in case the current position is further than 3m to the closest point on the path
            #print('head:{:6f}, proj_head:{:6f}, ye:{:6f}, widx:{:4d}, idx:{:4d}, wpthlen:{:4d}'.format(self.head,self.proj_head,self.ye,self.work_index,self.orig_index,len(self.working_path)))
            self.look_ahead = self.get_look_ahead(self.ye)
            if np.abs(self.ye) > self.MAX_DEV_FROM_PATH:
                rospy.logerr('Replan triggered')
                self.working_path = self.replan(vehicle_wp = self.current_wp, 
                                                current_wp_idx = self.orig_index, 
                                                original_path = self.original_path)
                self.work_index = 0             # Make sure to start from the first element of the augmented path

            # rospy.logdebug("Working path len is %d, current idx is %d", len(self.working_path), self.work_index)

            # Update controller 
            #rospy.logwarn(f"ye: {self.ye}")
            self.head, self.beta_hat  = self.ILOS(ye = self.ye,
                                                  beta_hat = self.beta_hat,
                                                  proj_heading = self.proj_head,
                                                  veh_speed = speed,
                                                  delta=self.look_ahead)
            
            if rospy.has_param("tgt_ilos_deg"):
                self.head = rospy.get_param("tgt_ilos_deg")*np.pi/180
        return self.mission_complete, self.head, self.ye, self.original_path[self.orig_index]

if __name__ == "__main__":
    # Create mission
    mission = NavigationTools.Mission(filename='mission.csv')
    # Use mission object to create a path to follow
    DubinsPath(mission=mission)
    #path_follower = PathFollower(mission=mission, path_creator=DubinsPath)
    