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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        # rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.current_frame_id = None
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # Record current pos and frame_id
        self.current_pose = msg.pose
        self.current_frame_id = msg.header.frame_id

        # Publish next LOOKAHEAD_WPS waypoints
        self.publish_final_waypoints()

    def waypoints_cb(self, msg):
        # TODO: Implement
        # base_waypoints are only published once, it needs to be stored in this node
        self.base_waypoints = msg.waypoints
        rospy.loginfo("base_waypoints received")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
        if self.current_pose is None or self.base_waypoints is None:
            return

        # Get nearest wapoint in front of ego vehicle
        closest_wp_idx = waypoint_updater_helper.get_closest_waypoint_index(self.current_pose, self.base_waypoints)
        rospy.loginfo("closest wp idx: %s", closest_wp_idx)

        # Get next LOOKAHEAD_WPS waypoints in front of ego vehicle
        next_final_wps_idx_start = closest_wp_idx
        next_final_wps_idx_end = min(len(self.base_waypoints), closest_wp_idx + LOOKAHEAD_WPS)
        next_final_wps = self.base_waypoints[next_final_wps_idx_start: next_final_wps_idx_end]
        rospy.loginfo("next final wp idx: %s-%s", next_final_wps_idx_start, next_final_wps_idx_end)
        rospy.loginfo("next final wp len: %s", len(next_final_wps))

        # Create and publish lane object to /final_waypoints topic
        next_lane = waypoint_updater_helper.generate_lane_object(self.current_frame_id, next_final_wps)
        self.final_waypoints_pub.publish(next_lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
