import numpy as np
import utm

def gps_to_local_frame(
    target_gps: tuple,
    origin_gps: tuple,
    frame_orientation_deg: float
) -> np.ndarray:
    """
    Convert a target GPS coordinate into local frame coordinates (in meters),
    The local frame's origin and orientation w.r.t UTM X axis are given.

    Parameters
    ----------
    target_gps : tuple (lat, lon)
        GPS coordinate of the target point.
    origin_gps : tuple (lat, lon)
        GPS coordinate of the origin of the local frame.
    frame_orientation_deg : float
        Orientation (degrees) of the local frame w.r.t. UTM X axis.
        - 0° means same orientation as UTM X.
        - Positive angles rotate counterclockwise.

    Returns
    -------
    np.ndarray
        2D vector [x_local, y_local] in meters, expressed in the local frame.
    """

    # Convert both GPS points to UTM coordinates (meters)
    target_utm = utm.from_latlon(target_gps[0], target_gps[1])
    origin_utm = utm.from_latlon(origin_gps[0], origin_gps[1])

    # Extract UTM X,Y
    target_xy = np.array([target_utm[0], target_utm[1]])
    origin_xy = np.array([origin_utm[0], origin_utm[1]])

    # Δ in UTM frame
    delta_utm = target_xy - origin_xy

    # Rotation matrix from UTM → local frame
    theta = np.deg2rad(frame_orientation_deg)
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Transform into local frame (inverse rotation)
    delta_local = R.T @ delta_utm

    return delta_local

if __name__ == '__main__':
    """ 
    For the sample test data:
        Viewed from the camera's perspective, the sample next waypoint is to the top
        and left of the camera frame with +y to the top and +x to the right 
        Separation: ~17.47m
    """
    next_waypoint_gps = (40.443026, -86.763256)
    camera_frame_origin_gps = (40.44286291645092, -86.76329222468132)
    camera_frame_orientation_deg = 90 # Same as boat orientation (in degrees) (camera's x axis is the same as boat heading.)
    
    x, y = gps_to_local_frame(
        target_gps=next_waypoint_gps,
        origin_gps=camera_frame_origin_gps,
        frame_orientation_deg=camera_frame_orientation_deg
    )
    print('x, y:', x, y)
    print(f'Separation: {round(np.hypot(x, y), 3)} (m)')