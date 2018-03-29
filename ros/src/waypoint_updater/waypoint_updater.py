#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
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
MAX_SPEED = 20.0 * MPH_TO_MPS  # in m/s

# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 20

# Number of waypoints to search before and/or after the nearest waypoint identified in previous cycle
INCR_WP_SEARCH_RANGE = 20


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=LOG_LEVEL)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.current_frame_id = None
        self.base_waypoints = None
        self.current_closest_wp_idx = None
        self.current_red_tl_wp_idx = None

        rospy.spin()

    def pose_cb(self, msg):
        # Record current pos and frame_id
        self.current_pose = msg.pose
        self.current_frame_id = msg.header.frame_id

        # Publish next LOOKAHEAD_WPS waypoints
        self.publish_final_waypoints()

    def waypoints_cb(self, msg):
        # base_waypoints are only published once, it needs to be stored in this node
        self.base_waypoints = msg.waypoints
        rospy.logdebug("%d base_waypoints received", len(self.base_waypoints))

    def traffic_cb(self, msg):
        self.current_red_tl_wp_idx = msg.data
        rospy.loginfo("red traffic light info received, near wp idx=%d", self.current_red_tl_wp_idx)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish_final_waypoints(self):
        if self.current_pose is not None and self.base_waypoints is not None:
            # Get closest waypoint in front of ego vehicle
            closest_wp_idx = waypoint_updater_helper.get_closest_wp_idx(
                pose=self.current_pose,
                waypoints=self.base_waypoints,
                prev_closest_wp_idx=self.current_closest_wp_idx,
                incr_wp_search_range=INCR_WP_SEARCH_RANGE)

            self.current_closest_wp_idx = closest_wp_idx
            rospy.logdebug("closest wp idx: %s", closest_wp_idx)

            # Get next LOOKAHEAD_WPS waypoints in front of ego vehicle
            next_wps_idx_start = closest_wp_idx
            next_wps_idx_end = min(len(self.base_waypoints), closest_wp_idx + LOOKAHEAD_WPS)
            next_wps = self.base_waypoints[next_wps_idx_start: next_wps_idx_end]
            rospy.logdebug("next final wp idx: %s-%s", next_wps_idx_start, next_wps_idx_end)
            rospy.logdebug("next final wp len: %s", len(next_wps))

            # Adjust target speed at each waypoint
            target_speed = MAX_SPEED  # if closest_wp_idx < 400 else 0 # test stop
            for waypoint in next_wps:
                waypoint.twist.twist.linear.x = target_speed

            # Create and publish lane object to /final_waypoints topic
            next_lane = waypoint_updater_helper.generate_lane_object(self.current_frame_id, next_wps)
            self.final_waypoints_pub.publish(next_lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
