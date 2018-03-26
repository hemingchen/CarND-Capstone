from math import cos, sin, sqrt
import tf
import rospy
import numpy as np
import math

from styx_msgs.msg import Lane


def distance(waypoints, p1, p2):
    """ Get total distance between two waypoints given their index"""
    gap = 0
    for i in range(p1, p2 + 1):
        gap += get_distance(waypoints[p1].pose.pose.position, waypoints[i].pose.pose.position)
        p1 = i
    return gap


def generate_lane_object(frame_id, waypoints):
    lane = Lane()
    lane.header.frame_id = frame_id
    lane.waypoints = waypoints
    lane.header.stamp = rospy.Time.now()
    return lane


def get_euler_angle(pose):
    return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])


def get_distance(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    return sqrt(dx * dx + dy * dy)


def waypoint_is_in_front_of_ego_veh(pose, waypoint):
    """
    Take one waypoint and current pose of ego vehicle, do a coordinate frame translation to move the origin to the
    current pos of ego vehicle and align the new frame's x-axis to ego vehicle x-axis.
    Args:
        pose (object) : A pose object
        waypoints (object) : A waypoint object
    Returns:
        bool : True if the waypoint is behind the car False if in front
    """
    _, _, ego_yaw = get_euler_angle(pose)
    ego_X = pose.position.x
    ego_Y = pose.position.y

    waypoint_X = waypoint.pose.pose.position.x
    waypoint_Y = waypoint.pose.pose.position.y

    shift_x = waypoint_X - ego_X
    shift_y = waypoint_Y - ego_Y

    return (shift_x * cos(0 - ego_yaw) - shift_y * sin(0 - ego_yaw)) > 0


def get_closest_waypoint_index(pose, waypoints):
    min_dist = float('inf')
    closest_wp_idx = 0
    ego_pos = pose.position

    for i, wp in enumerate(waypoints):
        dist = get_distance(ego_pos, wp.pose.pose.position)

        if dist < min_dist and waypoint_is_in_front_of_ego_veh(pose, wp):
            closest_wp_idx, min_dist = i, dist

    return closest_wp_idx


def get_next_waypoints(waypoints, i, n):
    """Returns a list of n waypoints ahead of the vehicle"""
    m = min(len(waypoints), i + n)
    return waypoints[i:m]


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given waypoints"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def calculateRCurve(coeffs, X):
    """
    calculates the radius of curvature
    Args:
        coeffs (vector) :polyfit coefficient of waypoints
        X (1D np array) : location to evaluate radius of curvature
    Return:
        radius_output (1D np array) : radius of curvature for X
    """
    if coeffs is None:
        return None
    coeffs_diff_1 = np.polyder(coeffs, 1)
    coeffs_diff_2 = np.polyder(coeffs, 2)

    radius_output = np.zeros(X.shape[0])
    for x_index in range(X.shape[0]):
        individual_x = X[x_index]
        radius = (1 + (np.polyval(coeffs_diff_1, individual_x) ** 2) ** 1.5) \
                 / np.absolute(np.polyval(coeffs_diff_2, individual_x))
        radius_output[x_index] = radius
    return radius_output