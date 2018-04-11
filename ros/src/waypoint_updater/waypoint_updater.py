#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

import waypoint_updater_helper

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOG_LEVEL = rospy.INFO

# Vehicle speed limit
MPH_TO_MPS = 0.44704
KMPH_TO_MPS = 0.27778
MAX_SPD = 40.0 * KMPH_TO_MPS  # m/s
MAX_SPD_CHG = 10  # m/s^2
# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 20
# Number of waypoints to search before and/or after the nearest waypoint identified in previous cycle
INCR_WP_SEARCH_RANGE = 20
# Travel time between waypoints, an rough estimate needed to calculate at which waypoint the deceleration should start.
DT_BTW_WPS = 0.02  # seconds
# Int32 value from tl_detector if no valid red traffic light in sight
NO_VALID_RED_TL_WP = -1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=LOG_LEVEL)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        # Publish final waypoints, which are the next sequence of waypoints the ego vehicle needs to follow.
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        # Publish ego vehicle waypoint idx, which is the idx of the base waypoints cloeses to ego vehicle.
        self.ego_veh_waypoint_pub = rospy.Publisher('/ego_veh_waypoint', Int32, queue_size=1)

        self.current_pose = None
        self.current_velocity = None
        self.current_frame_id = None
        self.base_waypoints = None
        self.base_waypoints_form_a_loop = False
        self.base_waypoints_len = None
        self.base_waypoints_last_idx = None
        self.current_closest_wp_idx = None
        self.next_red_tl_wp_idx = None
        self.next_decel_init_wp_idx = None

        # Identify numer of wps needed to a complete stop while traveling at MAX_SPD
        self.n_wps_to_stop_at_max_spd = waypoint_updater_helper.get_num_of_wps_during_const_spd_chg(
            v0=MAX_SPD, v1=0, a=-MAX_SPD_CHG, dt_btw_wps=DT_BTW_WPS)

        rospy.spin()

    def pose_cb(self, msg):
        # Record current pos and frame_id
        self.current_pose = msg.pose
        self.current_frame_id = msg.header.frame_id

        # Publish next LOOKAHEAD_WPS waypoints
        self.publish_final_waypoints()

    def current_velocity_cb(self, msg):
        # Record current linear and angular velocities of the ego vehicle
        self.current_velocity = msg.twist
        rospy.logdebug("current velocity received: %.2f m/s", self.current_velocity.linear.x)

    def waypoints_cb(self, msg):
        # base_waypoints are only published once, it needs to be stored in this node
        self.base_waypoints = msg.waypoints
        self.base_waypoints_len = len(self.base_waypoints)
        self.base_waypoints_last_idx = self.base_waypoints_len - 1
        rospy.logdebug("%d base_waypoints received", len(self.base_waypoints))

        # Check if base_waypoints form a loop
        self.base_waypoints_form_a_loop = waypoint_updater_helper.wps_forms_a_loop(self.base_waypoints)
        if self.base_waypoints_form_a_loop:
            rospy.logdebug("base_waypoints form a loop")
        else:
            rospy.logdebug("base_waypoints do not form a loop")

    def traffic_cb(self, msg):
        # Store next red traffic light stop line waypoint idx
        # If data=NO_VALID_RED_TL_WP, it means no need to stop, accelerate or maintain max spd.
        if msg.data == NO_VALID_RED_TL_WP:
            self.next_red_tl_wp_idx = None
            self.next_decel_init_wp_idx = None
            rospy.loginfo("no red traffic light info. maintain max spd or accel")

        # If red traffic light waypoint idx is received, update the latest point where a deceleration should start.
        else:
            self.next_red_tl_wp_idx = msg.data
            self.next_decel_init_wp_idx = self.next_red_tl_wp_idx - self.n_wps_to_stop_at_max_spd
            rospy.loginfo("red traffic light info received, near wp idx=%d", self.next_red_tl_wp_idx)
            rospy.loginfo("next decel init wp idx=%d", self.next_decel_init_wp_idx)

    def obstacle_cb(self, msg):
        pass

    def publish_final_waypoints(self):
        if self.current_pose is not None and self.base_waypoints is not None:
            #############################################################
            # 1. Get closest waypoint in front of ego vehicle
            #############################################################
            closest_wp_idx = waypoint_updater_helper.get_closest_wp_idx(
                pose=self.current_pose,
                all_waypoints=self.base_waypoints,
                prev_closest_wp_idx=self.current_closest_wp_idx,
                incr_wp_search_range=INCR_WP_SEARCH_RANGE)

            self.current_closest_wp_idx = closest_wp_idx
            rospy.loginfo("closest wp to ego veh idx=%s", closest_wp_idx)

            # Publish ego vehicle waypoint idx to /ego_veh_waypoint topic
            self.ego_veh_waypoint_pub.publish(Int32(self.current_closest_wp_idx))

            #############################################################
            # 2. Get next LOOKAHEAD_WPS waypoints in front of ego vehicle
            #############################################################
            next_wps_idx_start = closest_wp_idx
            next_wps_idx_end = min(self.base_waypoints_last_idx, closest_wp_idx + LOOKAHEAD_WPS - 1)
            if self.base_waypoints_form_a_loop and next_wps_idx_end == self.base_waypoints_last_idx:
                next_wps_idx_end = LOOKAHEAD_WPS - (next_wps_idx_end - next_wps_idx_start + 1) - 1
            rospy.logdebug("next final wp idx: %s-%s", next_wps_idx_start, next_wps_idx_end)

            # Create next waypoints from fresh
            next_wps = []
            if next_wps_idx_end >= next_wps_idx_start:
                for i in range(next_wps_idx_start, next_wps_idx_end + 1):
                    wp = Waypoint()
                    wp.pose.pose.position.x = self.base_waypoints[i].pose.pose.position.x
                    wp.pose.pose.position.y = self.base_waypoints[i].pose.pose.position.y
                    wp.pose.pose.position.z = self.base_waypoints[i].pose.pose.position.z
                    # For now, use default waypoint speed already defined by waypoint_loader.
                    wp.twist.twist.linear.x = self.base_waypoints[i].twist.twist.linear.x
                    next_wps.append(wp)
            else:
                # Tail of base_waypoints
                for i in range(next_wps_idx_start, self.base_waypoints_last_idx + 1):
                    wp = Waypoint()
                    wp.pose.pose.position.x = self.base_waypoints[i].pose.pose.position.x
                    wp.pose.pose.position.y = self.base_waypoints[i].pose.pose.position.y
                    wp.pose.pose.position.z = self.base_waypoints[i].pose.pose.position.z
                    # For now, use default waypoint speed already defined by waypoint_loader.
                    wp.twist.twist.linear.x = self.base_waypoints[i].twist.twist.linear.x
                    next_wps.append(wp)

                # Loops back to the beginning of the base_waypoints
                for i in range(0, next_wps_idx_end + 1):
                    wp = Waypoint()
                    wp.pose.pose.position.x = self.base_waypoints[i].pose.pose.position.x
                    wp.pose.pose.position.y = self.base_waypoints[i].pose.pose.position.y
                    wp.pose.pose.position.z = self.base_waypoints[i].pose.pose.position.z
                    # For now, use default waypoint speed already defined by waypoint_loader.
                    wp.twist.twist.linear.x = self.base_waypoints[i].twist.twist.linear.x
                    next_wps.append(wp)

            rospy.logdebug("next final wp len: %s", len(next_wps))

            #############################################################
            # 3. Get speed profile at each of next waypoints
            #############################################################
            # If all waypoints have been traversed, stop ego vehicle.
            if len(next_wps) < 2:
                for waypoint in next_wps:
                    waypoint.twist.twist.linear.x = 0

            # Otherwise, determine proper speed profile.
            else:
                # If approaching a traffic light, adjust speed properly.
                if self.next_red_tl_wp_idx >= 0 and self.next_decel_init_wp_idx >= 0:
                    next_wps = waypoint_updater_helper.update_next_wps_spd_profile(
                        next_wps=next_wps, next_wps_idx_start=next_wps_idx_start, next_wps_idx_end=next_wps_idx_end,
                        next_decel_init_wp_idx=self.next_decel_init_wp_idx, max_spd=MAX_SPD, max_spd_chg=MAX_SPD_CHG,
                        dt_btw_wps=DT_BTW_WPS)

                # Otherwise just use default spd, e.g. when tl_detector reports next_red_tl_wp_idx = -1.
                else:
                    pass

            #############################################################
            # 4. Create and publish lane object to /final_waypoints topic
            #############################################################
            next_lane = waypoint_updater_helper.generate_lane_object(self.current_frame_id, next_wps)
            self.final_waypoints_pub.publish(next_lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
