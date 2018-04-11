import math
from math import cos, sin, sqrt

import rospy
import tf
from styx_msgs.msg import Lane


def get_euler_angle(pose):
    """
    Get Euler angle from current position expressed as quaternion. Ego vehicle yaw will be needed from Euler angle.
    """
    return tf.transformations.euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w])


def get_distance(position1, position2):
    """
    Get distance between two positions
    """
    dx = position1.x - position2.x
    dy = position1.y - position2.y
    return sqrt(dx * dx + dy * dy)


def wp_is_in_front_of_ego_veh(pose, waypoint):
    """
    Take one waypoint and current pose of ego vehicle, do a coordinate frame translation to move the origin to the
    current pos of ego vehicle and align the new frame's x-axis to ego vehicle x-axis.

    Args:
        pose (Pose) : A pose object
        waypoints (Waypoint) : A waypoint object
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


def wps_forms_a_loop(waypoints):
    start_wp_position = waypoints[0].pose.pose.position
    end_wp_position = waypoints[-1].pose.pose.position
    dist = get_distance(start_wp_position, end_wp_position)
    # If distance between start and end waypoints is small, then there is a loop.
    if dist < 5:
        return True
    else:
        return False


def get_closest_wp_idx(pose, all_waypoints, prev_closest_wp_idx, incr_wp_search_range):
    """
    Get closest waypoint in front of ego vehicle
    """
    min_dist = float('inf')
    ego_pos = pose.position
    closest_wp_idx = 0
    use_prev_closest_wp = True if prev_closest_wp_idx else False
    wps_to_search = all_waypoints[prev_closest_wp_idx:prev_closest_wp_idx + incr_wp_search_range] \
        if use_prev_closest_wp else all_waypoints

    # Search for current nearest waypoint based on previous cycle result
    for i, wp in enumerate(wps_to_search):
        dist = get_distance(ego_pos, wp.pose.pose.position)
        if dist < min_dist and wp_is_in_front_of_ego_veh(pose, wp):
            closest_wp_idx, min_dist = i, dist

    # Add the offset back to get idx w.r.t the original base_waypoints
    if use_prev_closest_wp:
        closest_wp_idx += prev_closest_wp_idx

    # If found vehicle near the last waypoint, check if the waypoints are actually a loop.
    if closest_wp_idx == len(all_waypoints) - 1:
        # Check if loop exists where the first waypoint might be close to ego vehicle
        dist_to_1st_wp = get_distance(ego_pos, all_waypoints[0].pose.pose.position)
        if dist_to_1st_wp < min_dist:
            # Reset back to 1st waypoint
            closest_wp_idx, min_dist = 0, dist_to_1st_wp

    return closest_wp_idx


def get_num_of_wps_during_const_spd_chg(v0, v1, a, dt_btw_wps):
    """
    Get the min number of waypoints the ego veh needs to pass before completing the constant speed change.
    Assume v0, v1 in m/s, a in m/s^2 and a has sign.
    """
    t = (v1 - v0) / a
    n_wps = math.ceil(t / dt_btw_wps)

    return n_wps


def update_next_wps_spd_profile(next_wps, next_wps_idx_start, next_wps_idx_end, next_decel_init_wp_idx, max_spd,
                                max_spd_chg, dt_btw_wps):
    """
    Generate speed profile for ego vehicle for next sequence of waypoints to be published to /final_waypoints.
    """
    # Condition 1:
    # When next decel init wp is way ahead, just maintain max speed.
    if next_decel_init_wp_idx > next_wps_idx_end:
        return next_wps
    # Condition 2:
    # When next stop is close by, prepare to decelerate.
    else:
        for wp_rel_idx, wp_global_idx in enumerate(range(next_wps_idx_start, next_wps_idx_end)):
            if wp_global_idx < next_decel_init_wp_idx:
                # Maintain current waypoint speed, no need to change anything
                pass
            else:
                # Decelerate until zero speed at constant change rate
                current_wp_spd = next_wps[wp_rel_idx].twist.twist.linear.x
                expected_wp_spd = max_spd - max_spd_chg * (wp_global_idx - next_decel_init_wp_idx) * dt_btw_wps * 1.2
                next_wps[wp_rel_idx].twist.twist.linear.x = min(current_wp_spd, expected_wp_spd)
        return next_wps


def generate_lane_object(frame_id, waypoints):
    lane = Lane()
    lane.header.frame_id = frame_id
    lane.waypoints = waypoints
    lane.header.stamp = rospy.Time.now()
    return lane
