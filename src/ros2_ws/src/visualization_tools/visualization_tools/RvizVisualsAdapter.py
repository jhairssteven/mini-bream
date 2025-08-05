#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import tf_transformations as tf
import utm
import math as m

from sensor_msgs.msg import NavSatFix, Imu
from backseat_msgs.msg import Telem, Locg
from visualization_msgs.msg import Marker

def bearing2heading(bearing_deg):
    h = bearing_deg * m.pi / 180
    x = m.cos(h); y = m.sin(h)
    heading = m.atan2(x, y)
    if heading < 0:
        heading += 2 * m.pi
    return heading

def markerFromWypt(wypt, mkr_proto):
    x, y, yaw = wypt
    mkr_proto.header.stamp = rclpy.clock.Clock().now().to_msg()
    mkr_proto.pose = pose_from_waypoint([x, y, yaw])
    return mkr_proto

def getMarker(color=[1,0,0,1], type=Marker.ARROW, lwh=None):
    mkr = Marker()
    mkr.header.frame_id = "world"
    mkr.header.stamp = rclpy.clock.Clock().now().to_msg()
    mkr.type = type
    mkr.id = 0
    if lwh:
        l, w, h = lwh
        mkr.scale.x = float(l)
        mkr.scale.y = float(w)
        mkr.scale.z = float(h)
    else:
        if type == Marker.CYLINDER:
            mkr.scale.x = mkr.scale.y = 0.2; mkr.scale.z = 0.1
        elif type == Marker.ARROW:
            mkr.scale.x = 2.0; mkr.scale.y = mkr.scale.z = 0.2
        elif type == Marker.LINE_STRIP:
            mkr.scale.x = mkr.scale.y = mkr.scale.z = 0.1
        else:
            mkr.scale.x = mkr.scale.y = 0.5; mkr.scale.z = 0.1
    mkr.color.r, mkr.color.g, mkr.color.b, mkr.color.a = color
    return mkr

def gps_to_utm(lat, lon):
    u = utm.from_latlon(lat, lon)
    easts = float(u[0]); norths = float(u[1])
    zone = str(u[2]) + u[3]
    return easts, norths, zone

def pose_from_waypoint(xy_wp, is_yaw_in_rads=True):
    x, y, yaw = xy_wp
    if not is_yaw_in_rads:
        yaw *= m.pi / 180
    quat = tf.quaternion_from_euler(0, 0, yaw)
    p = Pose()
    p.position.x = float(x); p.position.y = float(y); p.position.z = 0.0
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p

def getPoseStampedfromXYWaypoint(xy_wp):
    ps = PoseStamped()
    ps.header.stamp = rclpy.clock.Clock().now().to_msg()
    ps.header.frame_id = "world"
    ps.pose = pose_from_waypoint(xy_wp)
    return ps

class RvizPath(Node):
    def __declare_parameters(self):
        self.declare_parameter("goal_lat", 40.448417)
        self.declare_parameter("goal_lon", -86.867750)
        self.declare_parameter("sim_enable", False)

    def __init__(self):
        super().__init__('rviz_path')
        self.__declare_parameters()
        init_lat = self.get_parameter("goal_lat").value
        init_lon = self.get_parameter("goal_lon").value
        self.ref_trajectory_pub = self.create_publisher(Path, '/ref_path', 1)
        self.waypts_pub = self.create_publisher(Marker, '/mission_waypts', 1)
        self.init_UTM_x, self.init_UTM_y, _ = gps_to_utm(init_lat, init_lon)

    def utm_waypts_to_coordinates(self, utm_waypts, UTM_init=None):
        base_x, base_y = UTM_init if UTM_init else (utm_waypts[0][0], utm_waypts[0][1])
        return [[wp[0]-base_x, wp[1]-base_y, wp[2]] for wp in utm_waypts]

    def publishWaypoints__thread(self, utm_waypts):
        import threading
        threading.Thread(target=self.pub_waypt_markers, args=(utm_waypts,), daemon=True).start()

    def pub_waypt_markers(self, utm_waypts):
        marker = self.get_marker_from_waypoints(utm_waypts)
        while rclpy.ok():
            self.waypts_pub.publish(marker)

    def get_marker_from_waypoints(self, utm_waypts):
        mkr = markerFromWypt([0,0,0], getMarker([1,1,0,1], type=Marker.LINE_STRIP))
        pts = []
        xy = self.utm_waypts_to_coordinates(utm_waypts, [self.init_UTM_x, self.init_UTM_y])
        for w in xy:
            p = Point(x=w[0], y=w[1], z=0.0)
            pts.append(p)
        mkr.points = pts[1:]
        return mkr

    def pub_ref_trajectory(self, utm_waypts):
        path = Path()
        xy = self.utm_waypts_to_coordinates(utm_waypts)
        for w in xy:
            path.poses.append(getPoseStampedfromXYWaypoint(w))
        path.header = path.poses[-1].header
        while rclpy.ok():
            self.ref_trajectory_pub.publish(path)

    def publishPath(self, utm_waypts):
        import threading
        threading.Thread(target=self.pub_ref_trajectory, args=(utm_waypts,), daemon=True).start()

class RvizPos(Node):
    def __declare_parameters(self):
        self.declare_parameter("goal_lat", 40.448417)
        self.declare_parameter("goal_lon", -86.867750)
        self.declare_parameter("sim_enable", True)
        self.declare_parameter("rst_trav_path", False)
        
    def __init__(self):
        super().__init__('rviz_pos')
        self.__declare_parameters()
        init_lon = self.get_parameter("goal_lon").value
        init_lat = self.get_parameter("goal_lat").value
        self.curr_lat = init_lat; self.curr_long = init_lon
        self.current_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.init_UTM_x, self.init_UTM_y, _ = gps_to_utm(init_lat, init_lon)
        self.past_poses = []
        self.ref_path = Path()
        self.ref_path.header.frame_id = 'world'
        self.heading_rad = 0; self.ilosHeading_rad = 0

        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 1)
        self.create_subscription(Float32, '/compass_bearing_deg', self.heading_callback, 1)
        self.create_subscription(Telem, '/telemetry', self.telem_callback, 1)
        self.create_subscription(Locg, '/way_gps', self.way_gps_callback, 1)
        self.create_subscription(Path, '/ref_path', self.ref_path_callback, 1)

        self.ilosHeadingMarkerPub = self.create_publisher(Marker, "/heading/marker/ilos", 2)
        self.pidrOutputMarkerPub = self.create_publisher(Marker, "/heading/marker/pidr_output", 2)
        self.currPoseMarkerPub = self.create_publisher(Marker, "/curr_marker", 2)
        self.goalPoseMarkerPub = self.create_publisher(Marker, "/goal_marker", 2)
        self.traversedPathPub = self.create_publisher(Path, "/traversed_path", 2)
        self.ilosHeadingPub = self.create_publisher(Float32, "/heading/value/ilos", 1)
        self.currAcuteHeadingPub = self.create_publisher(Float32, "/heading/value/curr_acute_heading", 1)
        self.PIDrErrorPub = self.create_publisher(Float32, "/pid/rotational/error", 1)

        self.ilosHeadingMarker = getMarker([0.5,1.0,0.25,1.0], Marker.ARROW, [25,0.2,0.2])
        self.pidrOutputMarker = getMarker([0.9,0.8,0.2,1.0], Marker.ARROW)
        self.goalPoseMarker = getMarker([0.0,0.0,1.0,1.0], Marker.ARROW)
        self.get_logger().info("'rviz_pose' node initialized...")

    def ref_path_callback(self, msg: Path):
        # optionally set origin from first pose
        pass

    def heading_callback(self, msg: Float32):
        b = msg.data * m.pi / 180
        x = m.cos(b); y = m.sin(b)
        ang = m.atan2(x, y)
        self.heading_rad = ang + (2*m.pi if ang < 0 else 0)

    def imu_callback(self, msg: Imu):
        self.current_orientation = msg.orientation

    def gps_callback(self, msg: NavSatFix):
        self.curr_lat = msg.latitude
        self.curr_long = msg.longitude
        self.current_pose_pub()

    def current_pose_pub(self):
        u = utm.from_latlon(self.curr_lat, self.curr_long)
        cx = u[0] - self.init_UTM_x
        cy = u[1] - self.init_UTM_y
        
        current_marker = getMarker([0.0,0.0,0.0,1.0], Marker.ARROW)
        current_pose = Pose(position=Point(x=cx, y=cy, z=0.0), orientation=self.current_orientation)
        current_marker.pose = current_pose
        self.currPoseMarkerPub.publish(current_marker)

        if self.get_parameter('rst_trav_path').value:
            self.ref_path.poses = self.ref_path.poses[-1:]
            self.set_parameters([rclpy.parameter.Parameter('rst_trav_path', rclpy.Parameter.Type.BOOL, False)])

        current_pose_stamped = PoseStamped()
        current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        current_pose_stamped.header.frame_id = "world"
        current_pose_stamped.pose = current_pose
        self.ref_path.poses.append(current_pose_stamped)
        self.traversedPathPub.publish(self.ref_path)

    def getCurrentXY(self):
        ux, uy, _ = gps_to_utm(self.curr_lat, self.curr_long)
        return [ux - self.init_UTM_x, uy - self.init_UTM_y]

    def way_gps_callback(self, msg: Locg):
        self.ilosHeading_rad = msg.dbear
        cu_x, cu_y = self.getCurrentXY()
        self.ilosHeadingMarker = markerFromWypt([cu_x, cu_y, self.ilosHeading_rad], self.ilosHeadingMarker)
        self.ilosHeadingMarkerPub.publish(self.ilosHeadingMarker)
        self.ilosHeadingPub.publish(Float32(data=self.ilosHeading_rad * 180 / m.pi))

    def telem_callback(self, msg: Telem):
        curr_x, curr_y = self.getCurrentXY()
        mkr = markerFromWypt([curr_x, curr_y, msg.desrot], self.pidrOutputMarker)
        self.pidrOutputMarkerPub.publish(mkr)

        emr = self.heading_rad - self.ilosHeading_rad
        if emr <= -m.pi: emr += 2*m.pi
        elif emr >= m.pi: emr -= 2*m.pi
        acute = (emr + self.ilosHeading_rad) * 180 / m.pi

        self.PIDrErrorPub.publish(Float32(data=emr * 180 / m.pi))
        self.currAcuteHeadingPub.publish(Float32(data=acute))

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = RvizPos()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
