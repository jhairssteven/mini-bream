import math as m
import numpy as np
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D
import time
from backseat.NavigationTools import *
import math
class PID:
    """PID Controller"""

    def __init__(self, p, i, d, wnd, setpoint):
        """Intialization/Constructor function that sets the proportional, integral, and derivative gains as well as the
        current time
        input:
            p = initialized proportional control gain [float]
            i = intialized integral control gain [float]
            d = initialized derivative control gain [float]
            wnd = intialized integral windup limit [float]
            setpoint = intialized PID setpoint to track [varying]
        output:
            void"""
        self.Kp = p  # set proportional gain
        self.Ki = i  # set the integral gain
        self.Kd = d  # set the derivative gain
        self.sample_time = 0.00  # set a default sample time for user to overwrite later if needed (real time update)
        self.windup_guard = wnd  # Initialize windup guard (max value ITerm can be)
        self.current_time = time.time()  # set the current time as the current time
        self.last_time = self.current_time  # set the initial last time to the current time
        self.last_error = 0.00  # initialize a variable to track previous error values
        self.error = 0.0  # initialize a variable to track current PID error
        self.ITerm = 0.0  # Initialize the term for integral gan
        self.PTerm = 0.0  # Initialize the term for Proportional gain
        self.DTerm = 0.0  # Initialize the term for derivative gain
        self.SetPoint = setpoint  # Initialize the desired value
        self.int_error = 0.0  # output the integration error
        self.output = 0.0  # Initialize the output variable
        self.feedback_value = 0.0  # Intialize the feedback value that will be updated by the ROS subscription service
        self.distto = 0.0  # Distance ot target
        self.delta_error = 0.0  # Create variable to output the delta_error
        self.delta_time = 0.0  # Create a variable to output the delta time

    def clear(self):
        """Clears PID computations and coefficients
        input:
            none (class)
        output:
            void"""
        # Clear outputs and goals
        self.SetPoint = 0.0
        self.last_error = 0.0
        self.delta_error = 0.0
        # Clear Terms
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        # Clear Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        # Clear Output
        self.output = 0.0

    def update(self):
        """Calculates PID value for given reference feedback
         math: u(t) = K_p e(t) + K_i/int_{0}^{t} e(t)dt + K_d {de}/{dt}
        input:
            none (class)
        output:
            void"""
        # self.error = self.feedback_value - self.SetPoint
        self.delta_error = self.error - self.last_error
        #print("self.delta_error",self.delta_error)
        self.delta_error =min(5,max(-5,self.delta_error ))
        # Dt Calc
        self.current_time = time.time()  # Set the current time
        self.delta_time = 0.1  # Find change in time = 1/ rospy.rate

        # Calculate Output
        # if delta_time >= self.sample_time:  # If enough time has elapsed (sample time) then update pid output
        # Calculate Terms
        self.PTerm = self.Kp * self.error  # Calculate Proportional term
        self.ITerm += self.error * self.delta_time  # Calculate Integral Term
        #print("self.ITerm: ", self.ITerm,  self.windup_guard)
        # Determine if the integral term is within the range of -windup_guard to + wind_guard
        if self.ITerm < -self.windup_guard:  # If it is less than negative guard
            self.ITerm = -self.windup_guard  # set integral term to  negative guard
        elif self.ITerm > self.windup_guard:  # IF it is more than guard
            self.ITerm = self.windup_guard  # set to guard

        # Calculate Differential term if time has passed
        self.DTerm = 0.0
        if self.delta_time > 0.0:  # If any time has passed
            self.DTerm = self.delta_error / self.delta_time  # calculate derivative term

        # Remember last time and last error for next calculation
        self.last_time = self.current_time  # Set last time as the time at which update last ran (i.e. now)
        self.last_error = self.error  # Save the error for use upon the next update
        #print("self.ITerm: ", self.PTerm,self.Ki * self.ITerm, self.Kd * self.DTerm)
        # Compile full output from terms and gain coefficients
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

# End PID Class Declaration


def get_dist(start_pose, goal_pose):
    """
    Euclidean distance
    :param start_pose: (x, y, yaw) meter
    :param goal_pose: (x, y, yaw) meter
    :return: in meter
    """
    return m.sqrt((start_pose.x - goal_pose.x) ** 2 + (start_pose.y - goal_pose.y) ** 2)


def norm(theta):
    """
    Normalize theta within (-pi, pi]
    :param theta: any theta in rad
    :return: in rad
    """
    result = theta
    while result > m.pi:
        result -= 2 * m.pi
    while result <= -m.pi:
        result += 2 * m.pi
    return result


def quadrant(theta):
    c = m.cos(theta)
    s = m.sin(theta)
    if (c > 0) ^ (s > 0):
        return 2 if c < 0 else 4
    else:
        return 1 if c > 0 else 3


def within(theta, theta_start, theta_end):
    """
    Judge if an angle is within certain half-plane range, all in (-pi, pi]
    angle on the range edge is not deemed as within here
    :param theta: angle to be judged in rad
    :param theta_start: start angle in rad
    :param theta_end: end angle in rad
    :return: within or not
    """
    assert abs(norm(theta_start - theta_end)) <= m.pi
    if theta_start == theta_end:
        return False
    if theta == theta_start or theta == theta_end:
        return False
    cur2start = norm(theta - theta_start)
    cur2end = norm(theta - theta_end)
    return ((cur2start > 0) ^ (cur2end > 0)) and abs(cur2start - cur2end) <= m.pi


def nearest_edge(theta, theta_start, theta_end):
    assert abs(norm(theta_start - theta_end)) < m.pi
    return theta_start if abs(norm(theta - theta_start)) < abs(norm(theta - theta_end)) else theta_end


def get_theta(start_pose, goal_pose):
    """
    Get arctan theta at start pose
    :param start_pose: (x, y, yaw)
    :param goal_pose: (x, y, yaw)
    :return: in rad
    """
    xs, ys = start_pose.x, start_pose.y
    xg, yg = goal_pose.x, goal_pose.y
    theta = m.atan2(yg - ys, xg - xs)
    return theta


def get_alpha(start_pose, goal_pose):
    """
    Get segment wrt start yaw in radian
    :param start_pose: (x, y, yaw)
    :param goal_pose: (x, y, yaw)
    :return: in rad
    """
    return norm(get_theta(start_pose, goal_pose) - start_pose.theta)


def get_beta(start_pose, goal_pose):
    """
    Get goal yaw wrt segment extension in radian
    :param start_pose: (x, y, yaw)
    :param goal_pose: (x, y, yaw)
    :return: in rad
    """
    return norm(norm(goal_pose.theta - start_pose.theta) - get_alpha(start_pose, goal_pose))


def deg2rad(deg):
    return deg / 180.0 * m.pi


def rad2deg(rad):
    return rad / m.pi * 180.0


def quat2rpy(quat):
    """
    Convert geometry_msgs::Quaternion to (roll, pitch, yaw)
    :param quat: quaternion
    :return: roll, pitch, yaw
    """
    q = [quat.x, quat.y, quat.z, quat.w]
    return euler_from_quaternion(q)


def get_yaw(quat):
    """
    Get yaw from quaternion
    :param quat:
    :return: yaw in rad
    """
    return quat2rpy(quat)[-1]


def distance_haversine(p1_lat, p1_lon, p2_lat, p2_lon, earth_radius=6371008):
    """This function calculates the shortest, "as the crow flies", distance between two points
    where point 1 is the starting point and point 2 is the end point.
    input:
        p1_lat = the latitude in degrees of the first point [deg]
        p1_lon = the longitude in degrees of the first point [deg]
        p2_lat = the latitude in degrees of the second point [deg]
        p2_lon = the longitude in degrees of the second point [deg]
        earth_rad = volumetric mean radius of the earth [m]
    output:
        dist = distance between p1 and p2 along the earth curvature [m]"""
    # Calculate the mean square sin of the change in lat an long
    sdlat2 = np.sin(np.radians(p1_lat - p2_lat) / 2.0) ** 2  # change in lat
    sdlon2 = np.sin(np.radians(p1_lon - p2_lon) / 2.0) ** 2  # change in long
    # Calculate a coefficient of the haversine formula
    a = sdlat2 + sdlon2 * np.cos(np.radians(p1_lat)) * np.cos(np.radians(p2_lat))
    # Calculate final distance
    dist = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a)) * earth_radius
    return dist  # return the distance in meters


def initial_bearing(p1_lat, p1_lon, p2_lat, p2_lon):
    """Bearing from the starting point (point 1) to the end point (point 2)
    input:
        p1_lat = the latitude in degrees of the first point
        p1_lon = the longitude in degrees of the first point
        p2_lat = the latitude in degrees of the second point
        p2_lon = the longitude in degrees of the second point
    output:
        bearing = compass heading to get from point 1 from point 2 [deg]
    ---------------------------------------------------------------
    For reference:
    phi = radial distance of a point from equator = latitude in radians
    lambda (lam) = radial distance from prime meridian = longitude in radians"""
    # Convert lats and longs to radians
    dlam = np.radians(p2_lon - p1_lon)  # delta or change in longitude in radians
    phi1 = np.radians(p1_lat)  # The latitude of point 1 in radians
    phi2 = np.radians(p2_lat)  # the latitude of point 2 in radians
    # Calculate offsets between points
    y = np.sin(dlam) * np.cos(phi2)  # Calculate the X or E-W offset
    x = np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(dlam)  # Calculate the Y or N-S offset
    # Calculate resulting angle through base trig
    bearing = np.degrees(np.arctan2(y, x)) % 360  # Calculate the bearing and convert to degrees
    return bearing  # return the bearing in degrees


def convert_2_bearing(angle):
    """This function converts a given angle from the path follower to a compass heading to use
    Input: Angle with respect to unit circle (+x axis) in radians (0 degrees east and increasing counter-clockwise
    Output: Compass heading in degrees (0 degrees north and increasing clockwise)"""
    # print(angle)
    x = m.cos(angle)
    y = m.sin(angle)
    bearingrad = m.atan2(x, y)
    # print(bearingRad)
    bearing = bearingrad * 180 / m.pi  # calculate bearing wrt compass heading (east -> north) and flip z axis
    if bearing < 0:  # if we got a negative bearing from atan2
        bearing = bearing + 360
    return bearing


def convert_2_angle(bearing):
    """This function converts a bearing from the onboad compass into radians
    Input: Compass heading in degrees (0 degrees north and increasing clockwise)
    Output: Angle with respect to unit circle (+x axis) in radians (0 degrees east and increasing counter-clockwise"""
    bearing = bearing * m.pi / 180  # convert to radians
    x = m.cos(bearing)
    y = m.sin(bearing)
    angle = m.atan2(x, y)
    if angle < 0:  # if we got a negative bearing from atan2
        angle = angle + 2 * m.pi
    return angle


def get_rho_alpha_beta_sphe(start_geopose, goal_geopose):
    """
    The function that returns all relative params (distance, heading errors) by spherical poses
    :param start_geopose: start geopose (lat, lon, yaw)
    :param goal_geopose: goal geopose (lat, lon, yaw)
    :return: Tuple (distance, start heading error, goal heading error)
    """
    start_lat, start_lon, start_yaw = start_geopose.lat, start_geopose.lon, start_geopose.yaw
    goal_lat, goal_lon, goal_yaw = goal_geopose.lat, goal_geopose.lon, goal_geopose.yaw
    rho = distance_haversine(start_lat, start_lon, goal_lat, goal_lon)

    bearing = initial_bearing(start_lat, start_lon, goal_lat, goal_lon)
    alpha = convert_2_angle(bearing) - start_yaw
    if alpha > 2 * m.pi:
        alpha -= 2 * m.pi
    if alpha > m.pi:
        alpha -= 2 * m.pi

    theta = start_yaw - goal_yaw
    if theta < -m.pi:
        theta = 2 * m.pi + theta
    if theta > m.pi:
        theta = 2 * m.pi - theta
    beta = - theta - alpha

    return rho, alpha, beta


def get_rho_alpha_beta_cart(start_pose, goal_pose):
    """
    The function that returns all relative params (distance, heading errors) by Cartesian poses
    :param start_pose: start cartesian pose (x, y, yaw)
    :param goal_pose: goal cartesian pose (x, y, yaw)
    :return: Tuple (distance, start heading error, goal heading error)
    """
    return get_dist(start_pose, goal_pose), get_alpha(start_pose, goal_pose), get_beta(start_pose, goal_pose)


def get_rho_alpha_beta_utm(xs, ys, thetas, xg, yg, thetag):
    """
    'Overload' function of the above cartesian version for utm info with heading
    :param xs: start utm pose x (m)
    :param ys: start utm pose y (m)
    :param thetas: start utm pose theta (rad)
    :param xg: goal utm pose x (m)
    :param yg: goal utm pose y (m)
    :param thetag: goal utm pose theta (rad)
    :return: Tuple (distance, start heading error, goal heading error)
    """
    start_pose = Pose2D(xs, ys, thetas)
    goal_pose = Pose2D(xg, yg, thetag)
    return get_rho_alpha_beta_cart(start_pose, goal_pose)


def get_goal_cart_pose(start_pose, rho, alpha, beta):
    """
    'Overload' function for start_pose of type Pose2D
    :param start_pose: start cartesian pose (x, y, yaw)
    :param rho: distance (of line segment) between 2 points in meter
    :param alpha: segment wrt start yaw in radian
    :param beta: goal yaw wrt segment extension in radian
    :return: goal cartesian pose (x, y, yaw)
    """
    return get_goal_utm_pose(start_pose.x, start_pose.y, start_pose.theta, rho, alpha, beta)


def get_goal_utm_pose(xs, ys, thetas, rho, alpha, beta):
    """
    The function that returns goal utm coordinate 2d pose by local polar info and the corresponding start utm 2d pose
    :param xs: start utm pose x (m)
    :param ys: start utm pose y (m)
    :param thetas: start utm pose theta (rad)
    :param rho: distance (of line segment) between 2 points in meter
    :param alpha: segment wrt start yaw in radian
    :param beta: goal yaw wrt segment extension in radian
    :return: goal utm pose (x, y, yaw)
    """
    theta_middle = norm(thetas + alpha)
    xg, yg = xs + rho * m.cos(theta_middle), ys + rho * m.sin(theta_middle)
    thetag = norm(theta_middle + beta)
    return xg, yg, thetag


def get_transformed_pose(xr, yr, thetar, x, y, theta):
    """
    Transform pose from reference pose frame to global frame
    Here reference frame usually means robot frame
    global frame usually means utm frame
    pose usually means target waypoint with heading
    :param xr:
    :param yr:
    :param thetar:
    :param x:
    :param y:
    :param theta:
    :return:
    """
    T = np.array([[m.cos(thetar), -m.sin(thetar), xr],
                  [m.sin(thetar), m.cos(thetar), yr],
                  [0, 0, 1]])
    homo_pose = np.array([[x],
                          [y],
                          [1]])
    new_pose = np.matmul(T, homo_pose)
    # print(new_pose)
    return new_pose[0][0], new_pose[1][0], norm(theta + thetar)


def get_twist(rho, alpha, beta, kr=0.1, ka=0.8, kb=-0.15, max_linear_x=1.0, max_angular_z=1.0):
    """
    The function that returns proportional controlled clipped cmd velocity
    :param rho: distance (of line segment) between 2 points in meter
    :param alpha: segment wrt start yaw in radian
    :param beta: goal yaw wrt segment extension in radian
    :param kr: P param for rho
    :param ka: P param for alpha
    :param kb: P param for beta
    :return: P controlled cmd vel
    :param max_linear_x: maximum allowed linear velocity in m/s
    :param max_angular_z: maximum allowed angular velocity in rad/s
    """
    t = Twist()
    linear_x = kr * rho
    t.linear.x = float(min(linear_x, max_linear_x) if linear_x >= 0 else max(linear_x, -max_linear_x))
    angular_z = ka * alpha + kb * beta
    t.angular.z = float(min(angular_z, max_angular_z) if angular_z > 0 else max(angular_z, -max_angular_z))
    return t


# if __name__ == "__main__":
#     # test case for within function
#     theta = -0.725768
#     theta = 1.57
#     theta = 0.78
#     theta = 2
#     theta = 3.14
#     theta = -2
#     theta = -1.57
#     theta = 0.071823
#     thetas = -1.756734352634008
#     thetag = 1.384858300955785
#     if within(theta, thetas, thetag):
#         print("Within!")
#     else:
#         print("NOT within!")
#
# if __name__ == "__main__":
#
#     start_pose = NavigationTools.Waypoint(gps_lat=0, gps_lon=0)
#     start_pose.x =284523.7456991684
#     start_pose.y=6266188.195589693
#     start_pose.theta = 1
#     xg, yg, thetag = get_goal_cart_pose(start_pose, 0.8, math.radians(-30), math.radians(30))
#     print(xg, yg, thetag)