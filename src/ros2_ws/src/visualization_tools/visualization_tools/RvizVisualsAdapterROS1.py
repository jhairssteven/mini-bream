#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import tf_transformations as tf
import utm
import math as m

from sensor_msgs.msg import NavSatFix
from backseat_msgs.msg import Telem, Locg
from visualization_msgs.msg import Marker

def bearing2heading(bearing_deg):
    """ Convert from bearing in degrees (0 north, +pi/2 east) to heading (0 east, +pi/2 north) in radians"""
    h = bearing_deg*m.pi/180  # convert to radians
    x = m.cos(h)
    y = m.sin(h)
    heading = m.atan2(x, y)
    if heading < 0:  # if we got a negative bearing from atan2
        heading = heading + 2*m.pi
    return heading

def markerFromWypt(wypt, market_prot):
    """ 
        Convert waypoint to UTM, get into local reference frame, build a Pose and return a Marker object with it
        wypt: waypoint in format [x (m), y (m), yaw (rad)]
        market_prot: Marker object prototype with some prepopulated 
                    (configuration fields such as color and size)
    """
    x, y, yaw = wypt
    #UTM_x, UTM_y, _ = gps_to_utm(lat, lon)
    #x, y = self.toLocalFrame(UTM_x, UTM_y)
    
    market_prot.header.stamp = rclpy.Time.now()
    market_prot.pose = pose_from_waypoint([x, y, yaw])

    return market_prot

def getMarker(color = [1, 0, 0, 1], type = Marker.ARROW, lwh = None):
        mkr = Marker()

        mkr.header.frame_id = "world"
        mkr.header.stamp = rclpy.Time.now()

        # Set the shape (Sphere in this case)
        mkr.type = type
        mkr.id = 0
        if lwh:
            [length, width, height] = lwh
            mkr.scale.x = length
            mkr.scale.y = width 
            mkr.scale.z = height
        else:
            # Set marker's dimensions
            if type == Marker.CYLINDER:
                mkr.scale.x = 0.2 # 20cm radious in x
                mkr.scale.y = 0.2 # 20cm radious in y
                mkr.scale.z = 0.1 # 10cm height
            elif type == Marker.ARROW:
                mkr.scale.x = 2 # length
                mkr.scale.y = 0.2 # width
                mkr.scale.z = 0.2 # height
            elif type == Marker.LINE_STRIP:
                mkr.scale.x = 0.1 # length
                mkr.scale.y = 0.1 # width
                mkr.scale.z = 0.1 # height
            else:
                mkr.scale.x = 0.5 # length
                mkr.scale.y = 0.5 # width
                mkr.scale.z = 0.1 # height
        
        # Set the color (red by default)
        mkr.color.r = color[0]
        mkr.color.g = color[1]
        mkr.color.b = color[2]
        mkr.color.a = color[3] # transparency
        
        return mkr

def gps_to_utm(lat, long):
        """This function will take a gps value and convert it to a utm coordinate and return the resulting
        norths and easts
        Input:
            lat = Latitude of vehicle in degrees North (negative is south)
            long = Latitude of vehcile in degrees East (negative is west)
        Output:
            Norths = the position of the vehicle in meters north of the origin of the current UTM zone
            Easts = the position of the vehicle in meters east of the origin o fthe current UTM Zone
            Zone = The current operational UTM zone
        """
        utmpos = str(utm.from_latlon(lat, long)).split(',')  # convert to utm [northings,eastings,zone #, "zone letter"]
        # break apart response
        easts = float(utmpos[0][1:])
        norths = float(utmpos[1][1:])
        zone = str(utmpos[2][1:]) + "T"
        return easts, norths, zone

def pose_from_waypoint(xy_wp, is_yaw_in_rads=True):
    """ Convert [x (m), y (m), yaw (rad)] waypoint to a PoseStamped ros message """
    import math as m
    x, y, yaw = xy_wp
    
    if not is_yaw_in_rads:
        yaw = yaw*m.pi/180

    quat = tf.quaternion_from_euler(0, 0, yaw)

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]

    return p

def getPoseStampedfromXYWaypoint(xy_wp):
        """ xy_wp: [x (m), y (m), yaw (rad)] """
        poseS = PoseStamped()
        poseS.header.stamp = rclpy.Time.now()
        poseS.header.frame_id = "world"
        poseS.pose = pose_from_waypoint(xy_wp)
        return poseS

class RvizPath:
    """ Publishes reference path (dubins) and mission_waypoints """
    #def __init__(self, init_lat=-33.722765834294044, init_lon=150.67398711024646): # VRX simulator
    #def __init__(self, init_lat=40.39224300046238, init_lon=-86.76039899999837): # kepner two points east
    #def __init__(self, init_lat=40.40256500290533, init_lon=-86.84538269042862): # kepner2
    #def __init__(self, init_lat=40.4024772644043, init_lon=-86.84527587890625): # kepner3 two points east
    #def __init__(self, init_lat=40.44755935716018, init_lon=-86.8684616088858): # lakefield
    #def __init__(self, init_lat=40.44771957397461, init_lon=-86.86841583251953): # lakefield may23
    #def __init__(self, init_lat=40.392243, init_lon=-86.760399): # kepner
    #def __init__(self, init_lat=40.402611341555364, init_lon=-86.84546864622155): # kepnerEntrance
    #def __init__(self, init_lat=40.40267562866211, init_lon=-86.84526824951172): # when mini-bream pos on lab
    def __init__(self, init_lat=rclpy.get_param("goal_lat"), init_lon=rclpy.get_param("goal_lon")):
        #rclpy.init_node('state_pub')
        self.ref_trajectory_pub = rclpy.Publisher('/ref_path', Path, queue_size=1)
        self.waypts_pub = rclpy.Publisher('/mission_waypts', Marker, queue_size=1)
        
        self.init_UTM_x, self.init_UTM_y, _ = gps_to_utm(init_lat, init_lon)

    def utm_waypts_to_coordinates(self, utm_waypts, UTM_init=None):
        """ Return all utm coordinates with respect to the first UTM coordinate in array 
        Input: 
            utm_waypts: format [x (m), y (m), yaw (rad)]
            UTM_init: [init_UTM_x, init_UTM_y] initial UTM to set origin
        """
        xy_waypts = []
        if UTM_init:
            utm_init_x, utm_init_y = UTM_init
        else:
            utm_init_x = utm_waypts[0][0]
            utm_init_y = utm_waypts[0][1]
        
        for wp in utm_waypts:
            xy_waypts.append([wp[0]-utm_init_x, wp[1]-utm_init_y, wp[2]])
        return xy_waypts

    def publishWaypoints__thread(self, utm_waypts):
        import threading
        
        pub_thread = threading.Thread(
            target=self.pub_waypt_markers, args=(utm_waypts,))
        
        pub_thread.daemon = True
        pub_thread.start()
        

    def pub_waypt_markers(self, utm_waypts):
        marker = self.get_marker_from_waypoints(utm_waypts)
        
        #rate = rclpy.Rate(1)

        while not rclpy.is_shutdown():
            
            self.waypts_pub.publish(marker)
            #rclpy.loginfo("Published mission waypoints")
            #rate.sleep()
        
    def get_marker_from_waypoints(self, utm_waypts):
        """
            Converts the UTM_waypts into an equivalent MarkerArray message.
        Args:
            utm_waypts: [[UTM_x, UTM_y, yaw],] waypoints 
        """
        
        marker = markerFromWypt([0,0,0], getMarker([1, 1, 0, 1], type=Marker.LINE_STRIP))
        points = []

        xy_waypts = self.utm_waypts_to_coordinates(utm_waypts, [self.init_UTM_x, self.init_UTM_y])
        for xy_wp in xy_waypts:
            cp = Point()
            cp.x, cp.y, _ = xy_wp
            cp.z = 0.0
            points.append(cp)
        marker.points = points[1:]
        return marker

    def pub_ref_trajectory(self, utm_waypts):
        ref_path = Path()
        xy_waypts = self.utm_waypts_to_coordinates(utm_waypts)
        rclpy.logerr(xy_waypts)
        for xy_wp in xy_waypts:
            pose = getPoseStampedfromXYWaypoint(xy_wp)
            ref_path.poses.append(pose)
        
        ref_path.header = ref_path.poses[-1].header
        
        rate = rclpy.Rate(1)
        while not rclpy.is_shutdown():
            self.ref_trajectory_pub.publish(ref_path)
            #rclpy.loginfo("Published reference path with {} waypts".format(len(utm_waypts)))
            rate.sleep()    
        
    def publishPath(self, utm_waypts):
        """ 
        Publish path on its own thread to prevent main thread blocking
        utm_waypts: array of UTM waypts [x(m), y(m), yaw (rad)] """
        import threading
        
        pub_thread = threading.Thread(
            target=self.pub_ref_trajectory, args=(utm_waypts,))
        
        pub_thread.daemon = True
        pub_thread.start()

def pathTester():
    rvizAdapter = RvizPath()
    rate = rclpy.Rate(1)
    
    from test_waypts import test_waypts
    import math
    test_waypts = test_waypts[:3]
    
    while not rclpy.is_shutdown():
        rvizAdapter.pub_ref_trajectory(test_waypts)
        rate.sleep()
        
    """ xy_waypts = rvizAdapter.utm_waypts_to_coordinates(test_waypts)
    for i in range(len(test_waypts)):
        print(test_waypts[i], " -- ", xy_waypts[i]) """


class RvizPos:
    """ Publishes current heading and goal heading. Also plots traversed path. """
    #def __init__(self, init_lat=-33.722765834294044, init_lon=150.67398711024646): # VRX simulator
    #def __init__(self, init_lat=40.39224300046238, init_lon=-86.76039899999837): # kepner two points east
    #def __init__(self, init_lat=40.40256500290533, init_lon=-86.84538269042862): # kepner2
    #def __init__(self, init_lat=40.4024772644043, init_lon=-86.84527587890625): # kepner3 two points east
    #def __init__(self, init_lat=40.44755935716018, init_lon=-86.8684616088858): # lakefield
    #def __init__(self, init_lat=40.44771957397461, init_lon=-86.86841583251953): # lakefield may23
    #def __init__(self, init_lat=40.392243, init_lon=-86.760399): # kepner
    #def __init__(self, init_lat=40.402611341555364, init_lon=-86.84546864622155): # kepnerEntrance
    #def __init__(self, init_lat=40.40267562866211, init_lon=-86.84526824951172): # when mini-bream pos on lab
    def __init__(self, init_lat=rclpy.get_param("goal_lat"), init_lon=rclpy.get_param("goal_lon")):
        #self.init_UTM_x = 284479.67
        #self.init_UTM_y = 6266160.29
        self.curr_lat =  init_lat
        self.curr_long = init_lon
        #self.init_UTM_x, self.init_UTM_y, _ = [513111.0998706102, 4472418.445489976, None] #gps_to_utm(init_lat, init_lon)
        self.init_UTM_x, self.init_UTM_y, _ = gps_to_utm(init_lat, init_lon)

        self.past_poses = []
        self.past_path_pub_freq = 3
        self.counter = 0
        self.ref_path = Path()
        self.ref_path.header.frame_id = 'world'
        self.heading_rad = 0
        self.ilosHeading_rad = 0
        #self.init_UTM_x = None
        #self.init_UTM_y = None
        self.i = 0
        rclpy.init_node('rviz_pos')

        # Subscribers

        if rclpy.get_param("sim_enable"):
            rclpy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback, queue_size=1)
        else:
            rclpy.Subscriber('/ublox_gps/fix', NavSatFix, self.gps_callback, queue_size=1)
            
        rclpy.Subscriber('/compass_bearing_deg', Float32, self.heading_callback) # TODO: change to the one published by VRX_topic_adapter
        rclpy.Subscriber('/telemetry', telem, self.telem_callback)
        rclpy.Subscriber('/ref_path', Path, self.ref_path_callback)
        rclpy.Subscriber('/way_gps', locg, self.way_gps_callback) # TODO: change to the value published on telemetry or stat
        
        # Marker publishers
        self.ilosHeadingMarkerPub = rclpy.Publisher("/heading/marker/ilos", Marker, queue_size=2)
        self.pidrOutputMarkerPub = rclpy.Publisher("/heading/marker/pidr_output", Marker, queue_size=2)
        self.currPoseMarkerPub = rclpy.Publisher("/curr_marker", Marker, queue_size=2)
        self.goalPoseMarkerPub = rclpy.Publisher("/goal_marker", Marker, queue_size=2)
        self.traversedPathPub = rclpy.Publisher("/traversed_path", Path, queue_size=2)
        
        # Number-only publishers
        self.ilosHeadingPub = rclpy.Publisher("/heading/value/ilos", Float32, queue_size=1)
        self.currAcuteHeadingPub = rclpy.Publisher("/heading/value/curr_acute_heading", Float32, queue_size=1)
        #self.PIDrInputPub = rclpy.Publisher("/pid/rotational/input", Float32, queue_size=1)
        self.PIDrErrorPub = rclpy.Publisher("/pid/rotational/error", Float32, queue_size=1)
        #self.PIDrSystemResponsePub = rclpy.Publisher("/pid/rotational/system_response", Float32, queue_size=1)

        self.ilosHeadingMarker = getMarker(color=[0.5, 1.0, 0.25, 1.0], type=Marker.ARROW, lwh=[25, 0.2, 0.2])
        self.pidrOutputMarker = getMarker(color=[0.9, 0.8, 0.2, 1.0], type=Marker.ARROW)
        #self.currPoseMarker = getMarker(color=[0.0, 0.0, 0.0, 1.0], type=Marker.ARROW) # White
        self.goalPoseMarker = getMarker(color=[0.0, 0.0, 1.0, 1.0], type=Marker.ARROW) # Blue

    def ref_path_callback(self, data):
        """ Get the path's first [x, y] pose values and set them as origin for local reference frame """
        ini_pose = data.poses[0]
        #self.init_UTM_x, self.init_UTM_y, _ = gps_to_utm(40.40267562866211, -86.84526824951172) # gps_to_utm(-33.722765834294044, 150.67398711024646) # simu
    
    def heading_callback(self, data):
        """ Output: 0 East, 90° North"""

        def convert_2_angle(bearing_deg):
            """This function converts a bearing from the onboad compass into radians
            Input: Compass heading in degrees (0 degrees north and increasing clockwise)
            Output: Angle with respect to unit circle (+x axis) in radians (0 degrees east and increasing counter-clockwise"""
            bearing_deg = bearing_deg*m.pi/180  # convert to radians
            x = m.cos(bearing_deg)
            y = m.sin(bearing_deg)
            angle = m.atan2(x, y)
            if angle < 0:  # if we got a negative bearing from atan2
                angle = angle + 2*m.pi
            return angle
        
        bearing_deg = data.data # comming as bearing
        self.heading_rad = convert_2_angle(bearing_deg)

    def gps_callback(self, data):
        curr_lat = data.latitude
        curr_long = data.longitude
        self.heading_rad = self.heading_rad
        
        self.curr_lat = curr_lat
        self.curr_long = curr_long
        self.current_pose_pub()
    
    def current_pose_pub(self):
        if (self.curr_lat is not None):        
            utm_result = utm.from_latlon(self.curr_lat, self.curr_long)
            curr_x = utm_result[0] - self.init_UTM_x # 513111.0998706102
            curr_y = utm_result[1] - self.init_UTM_y # 4472418.445489976
        else:
            curr_x = 0
            curr_y = 0
        #rclpy.loginfo("(x,y, yaw (rads)): %s, %s, %s", curr_x, curr_y, curr_heading)
        #print(curr_UTM_x, curr_UTM_y)
        cpm = getMarker(color=[0.0, 0.0, 0.0, 1.0], type=Marker.ARROW) # White
        self.currPoseMarker = self.markerFromWypt(
            [curr_x, curr_y, self.heading_rad], cpm)
        
        #rclpy.loginfo("curr (x, y): %s, %s", round(curr_x, 3), round(curr_y, 3))
        self.currPoseMarkerPub.publish(self.currPoseMarker)

        #pose = getPoseStampedfromXYWaypoint([curr_x, curr_y, self.heading_rad])
        pose = PoseStamped()
        pose.header.stamp = rclpy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = curr_x
        pose.pose.position.y = curr_y
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0

        self.ref_path.poses.append(pose)
        rst_trav_path = rclpy.get_param("rst_trav_path") if rclpy.has_param("rst_trav_path") else False
        if rst_trav_path:
            self.ref_path.poses = self.ref_path.poses[-1:]
            rclpy.set_param("rst_trav_path", False)
        else:
            self.ref_path.poses = self.ref_path.poses
        #self.ref_path.header = pose.header
        self.traversedPathPub.publish(self.ref_path)
    
    def getCurrentXY(self):
        """ Get the current [X, Y] position from current GPS value """
        curr_UTM_x, curr_UTM_y, _ = gps_to_utm(self.curr_lat, self.curr_long)
        curr_x = curr_UTM_x - self.init_UTM_x
        curr_y = curr_UTM_y - self.init_UTM_y
        
        return [curr_x, curr_y]
    
    def way_gps_callback(self, data):
        self.ilosHeading_rad = data.dbear
        #self.ilosHeading_rad = bearing2heading(bearing_deg=data.deshead) # heading in radians
        cu_x, cu_y = self.getCurrentXY()
        cu_wypt = [cu_x, cu_y, self.ilosHeading_rad]
        
        # Get and publish current poses as Markers
        self.ilosHeadingMarker = self.markerFromWypt(cu_wypt, self.ilosHeadingMarker)
        self.ilosHeadingMarkerPub.publish(self.ilosHeadingMarker)
        self.ilosHeadingPub.publish(self.ilosHeading_rad*180/m.pi)

    
    def telem_callback(self, data):
        if self.init_UTM_x == None and self.init_UTM_y == None:
            #rclpy.logerror("Waiting for /ref_path to set origin frame coordinates")
            rclpy.logerr("init_UTM_x and init_UTM_y have not yet been initialized. Waiting for /ref_path")
            return
        
        """ goal_lat = data.glat
        goal_long = data.glong
        desired_bearing = data.dbear*m.pi/180 # same as Ilos bearing
        curr_lat = data.lat
        curr_long = data.long
        curr_heading = data.hed """
        

        pidr_desHeading = data.desrot
        curr_x, curr_y = self.getCurrentXY()

        # Get and publish current and goal poses as Markers
        marker_msg = self.markerFromWypt([curr_x, curr_y, pidr_desHeading], self.pidrOutputMarker)
        self.pidrOutputMarkerPub.publish(marker_msg)


        emr = self.heading_rad - self.ilosHeading_rad

        if emr <= -m.pi:  # if large neg offset then use sup. angle
            pidr_error_rad = (emr + 2*m.pi)  # adjust for the smaller CCW angle
        elif emr >= 180.0:  # if large pos offset then use sup. angle
            pidr_error_rad = (emr - 2*m.pi)  # adjust for the smaller CW angle
        else:
            pidr_error_rad = emr
        
        # Current heading is obtained this way to always get the smallest angle between 
        # actual current heading (0°-360°) and ilos heading (0°-360°)
        # ilos could be 359° and current_heading 5°. The simple subsctraction would yield
        # 354° but actual angle difference between vectors is only 6°.
        acute_curr_heading_deg = (pidr_error_rad + self.ilosHeading_rad)*180/m.pi
        
        
        self.PIDrErrorPub.publish(pidr_error_rad*180/m.pi)
        self.currAcuteHeadingPub.publish(acute_curr_heading_deg)
        #self.PIDrSystemResponsePub.publish(acute_curr_heading_deg)

    
    def markerFromWypt(self, wypt, market_prot):
        """ 
            Convert waypoint to UTM, get into local reference frame, build a Pose and return a Marker object with it
            wypt: waypoint in format [x (m), y (m), yaw (rad)]
            market_prot: Marker object prototype with some prepopulated 
                        (configuration fields such as color and size)
        """
        x, y, yaw = wypt
        #UTM_x, UTM_y, _ = gps_to_utm(lat, lon)
        #x, y = self.toLocalFrame(UTM_x, UTM_y)
        
        market_prot.header.stamp = rclpy.Time.now()
        market_prot.pose = pose_from_waypoint([x, y, yaw])

        return market_prot

    def toLocalFrame(self, UTM_x, UTM_y):
        """ Get give UTM coordinates with respect to a fixed origin point """
        x = UTM_x - self.init_UTM_x
        y = UTM_y - self.init_UTM_y
        return x, y
    
    def getArrowMarker(self, color = [0.0, 0.0, 1.0, 1.0]):
        """ color: Marker's color with format [r, g, b, transparency] """
        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rclpy.Time.now()

        # Set the shape (Sphere in this case)
        marker.type = Marker.CYLINDER
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1

        # Set the color (green in this case)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Set the position of the marker
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        return marker
    
    def run(self):
        """ rate = rclpy.Rate(1)
        while not rclpy.is_shutdown():
            self.current_pose_pub()
            rate.sleep() """
        rclpy.spin()

def rvizPos_tester():
    #gps_to_utm(40.40267562866211, -86.84526824951172) # gps_to_utm(-33.722765834294044, 150.67398711024646) # lab coordinates
    rvizPos = RvizPos()
    rvizPos.run()


if __name__ == '__main__':
    # pathTester()
    rvizPos_tester()
