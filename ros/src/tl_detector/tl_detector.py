#!/usr/bin/env python

import yaml
from math import sqrt

import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from styx_msgs.msg import Lane
from styx_msgs.msg import TrafficLightArray, TrafficLight

import tl_detector_helper
from light_classification.tl_classifier import TLClassifier

LOG_LEVEL = rospy.INFO

# Min number of successive occurrence before a traffic light detection result is used
STATE_COUNT_THRESHOLD = 3
# If vehicle is within this distance to a traffic light, identify light status and adjust proposed speed if needed
TL_VISIBLE_DIST = 100
# Use broadcasted traffic light status for unit testing
USE_BROADCASTED_TL_STATES = False
# Use camera to detect traffic light status for comprehensive testing and production
USE_CAMERA_DETECTED_TL_STATES = not USE_BROADCASTED_TL_STATES
# Number of waypoints to search before and/or after the nearest waypoint identified in previous cycle
INCR_WP_SEARCH_RANGE = 20
# Adjust execution rate to reduce latency between vm and simulator on host
TL_DETECTOR_RATE = 3
# Int32 value from tl_detector if no valid red traffic light in sight
NO_VALID_RED_TL_WP = -1


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=LOG_LEVEL)

        config_string = rospy.get_param("/traffic_light_config")

        self.config = yaml.load(config_string)

        self.current_pose = None
        self.current_closest_wp_idx = None
        self.base_waypoints = None
        self.base_waypoints_form_a_loop = False
        self.base_waypoints_len = None
        self.base_waypoints_last_idx = None
        self.current_camera_image = None
        self.current_tl_state = None

        self.last_n_tl_state_preds = []
        self.tl_wp_indices = None
        self.traffic_lights = None

        self.tl_pred_data_source = None
        if USE_CAMERA_DETECTED_TL_STATES:
            self.tl_pred_data_source = "(from camera)"
        if USE_BROADCASTED_TL_STATES:
            self.tl_pred_data_source = "(from broadcast)"

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=5)
        rospy.Subscriber('/ego_veh_waypoint', Int32, self.ego_veh_waypoint_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=5)

        # Had to use loop here to reduce load on vm, otherwise latency between vm and simulator will break the control.
        self.loop()

    def loop(self):
        rate = rospy.Rate(TL_DETECTOR_RATE)
        while not rospy.is_shutdown():
            # Publish /vehicle/traffic_lights at TL_DETECTOR_RATE frequency
            self.publish_traffic_light_wp_info()

            rate.sleep()

    def pose_cb(self, msg):
        # Record current pos
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        # base_waypoints are only published once, it needs to be stored in this node
        self.base_waypoints = msg.waypoints
        self.base_waypoints_len = len(self.base_waypoints)
        self.base_waypoints_last_idx = self.base_waypoints_len - 1
        rospy.logdebug("%d base_waypoints received", len(self.base_waypoints))

        # Check if base_waypoints form a loop
        self.base_waypoints_form_a_loop = tl_detector_helper.wps_forms_a_loop(self.base_waypoints)
        if self.base_waypoints_form_a_loop:
            rospy.logdebug("base_waypoints form a loop")
        else:
            rospy.logdebug("base_waypoints do not form a loop")

    def traffic_cb(self, msg):
        self.traffic_lights = msg.lights
        rospy.logdebug("%d traffic lights' info received", len(self.traffic_lights))

        # Find out closest waypoints to each traffic light's stop line, it will run only once.
        self.update_closest_wp_idx_to_traffic_lights()

    def image_cb(self, msg):
        # Cache current camera image
        self.current_camera_image = msg
        rospy.logdebug("camera image received")

    def ego_veh_waypoint_cb(self, msg):
        self.current_closest_wp_idx = msg.data
        rospy.logdebug("closest wp to ego veh idx=%s", self.current_closest_wp_idx)

    def get_closest_waypoint_near_position(self, position):
        min_dist = float('inf')
        closest_wp_idx = -1

        # Search for base waypoint nearest to given pose
        for i, wp in enumerate(self.base_waypoints):
            dist = sqrt((wp.pose.pose.position.x - position.x) ** 2 +
                        (wp.pose.pose.position.y - position.y) ** 2)
            if dist < min_dist:
                closest_wp_idx, min_dist = i, dist

        return closest_wp_idx

    def get_traffic_light_state_from_broadcast(self, tl_idx):
        return self.traffic_lights[tl_idx].state

    def get_traffic_light_state_from_image(self):
        if self.current_camera_image is not None:
            # Get traffic light classification result
            cv_image = self.bridge.imgmsg_to_cv2(self.current_camera_image, "bgr8")

            return self.light_classifier.get_classification(cv_image)

    def update_closest_wp_idx_to_traffic_lights(self):
        # Find out closest waypoints to each traffic light's stop line, do it only once.
        if self.traffic_lights is not None and self.base_waypoints is not None and self.tl_wp_indices is None:
            tl_wp_indices = []
            for i_tl, tl in enumerate(self.traffic_lights):
                tl_stopline_pos = Point()
                tl_stopline_pos.x = self.config["stop_line_positions"][i_tl][0]
                tl_stopline_pos.y = self.config["stop_line_positions"][i_tl][1]

                closest_wp_idx = self.get_closest_waypoint_near_position(tl_stopline_pos)
                tl_wp_indices.append(closest_wp_idx)
                rospy.logdebug("traffic light idx=%s-->closest wp idx=%d", i_tl, closest_wp_idx)

            self.tl_wp_indices = tl_wp_indices
            rospy.logdebug("closest wps to traffic light stoplines identified")

    def get_next_traffic_light_info(self):
        # Get position of next traffic light (nearest waypoint to traffic light stop line)
        if self.current_closest_wp_idx is not None and \
                self.traffic_lights is not None and \
                self.base_waypoints is not None and \
                self.tl_wp_indices is not None:
            next_tl_wp_idx = None
            candidate_tl_wp_indices = [tl_wp_idx for tl_wp_idx in self.tl_wp_indices if
                                       tl_wp_idx > self.current_closest_wp_idx]
            # When approaching the end of route and no more traffic light
            if len(candidate_tl_wp_indices) == 0:
                # Loop back to starting point if base_waypoints form a loop
                if self.base_waypoints_form_a_loop:
                    next_tl_wp_idx = min(self.tl_wp_indices)
                # Route ends and no traffic light
                else:
                    next_tl_wp_idx = None
            # Not approaching the end of route
            else:
                next_tl_wp_idx = min(candidate_tl_wp_indices)

            if next_tl_wp_idx is not None:
                next_tl_wp = self.base_waypoints[next_tl_wp_idx]
                next_tl_idx = self.tl_wp_indices.index(next_tl_wp_idx)
                dist_to_next_tl = sqrt((next_tl_wp.pose.pose.position.x - self.current_pose.position.x) ** 2 +
                                       (next_tl_wp.pose.pose.position.y - self.current_pose.position.y) ** 2)

                return next_tl_idx, next_tl_wp_idx, dist_to_next_tl
            else:
                return None, None, None
        else:
            return None, None, None

    def traffic_light_state_prediction_filter(self, next_tl_state_raw_pred):
        # Maintain a FIFO stack with length STATE_COUNT_THRESHOLD
        self.last_n_tl_state_preds.append(next_tl_state_raw_pred)
        if len(self.last_n_tl_state_preds) > STATE_COUNT_THRESHOLD:
            self.last_n_tl_state_preds.pop(0)

        # If all elements are the same and different than current state, declare a new state.
        if len(set(self.last_n_tl_state_preds)) == 1 and next_tl_state_raw_pred != self.current_tl_state:
            self.current_tl_state = next_tl_state_raw_pred

    def publish_traffic_light_wp_info(self):
        # Find out next traffic light:
        # If red and close to ego vehicle, publish its stop line coord (actually the coord of the waypoint nearest
        # to the stop line) to /traffic_waypoint topic
        # If not red, do not publish anything
        next_tl_idx, next_tl_wp_idx, dist_to_next_tl = self.get_next_traffic_light_info()

        if next_tl_idx is not None and next_tl_wp_idx is not None and dist_to_next_tl is not None:
            next_tl_state_raw_pred = None

            # For unit test, use broadcasted traffic light state.
            if USE_BROADCASTED_TL_STATES:
                next_tl_state_raw_pred = self.get_traffic_light_state_from_broadcast(next_tl_idx)
            # For comprehensive test and production, use camera data and ml to determine traffic light state.
            if USE_CAMERA_DETECTED_TL_STATES:
                next_tl_state_raw_pred = self.get_traffic_light_state_from_image()
            rospy.logdebug("next traffic light idx=%d, dist=%.2f, raw pred state=%s%s",
                           next_tl_idx, dist_to_next_tl,
                           self.get_traffic_light_state_name(next_tl_state_raw_pred),
                           self.tl_pred_data_source)

            # Declare prediction result only if it has repeated STATE_COUNT_THRESHOLD times
            self.traffic_light_state_prediction_filter(next_tl_state_raw_pred)
            rospy.loginfo("next traffic light idx=%d, dist=%.2f, state=%s%s",
                          next_tl_idx, dist_to_next_tl,
                          self.get_traffic_light_state_name(self.current_tl_state),
                          self.tl_pred_data_source)

            # Publish actual traffic light waypoint idx only if in red and close to ego vehicle
            if self.current_tl_state == TrafficLight.RED and dist_to_next_tl < TL_VISIBLE_DIST:
                self.upcoming_red_light_pub.publish(Int32(next_tl_wp_idx))
                rospy.loginfo("next traffic light info published")
            # Publish -1 in case no need to stop at this moment
            else:
                self.upcoming_red_light_pub.publish(Int32(NO_VALID_RED_TL_WP))
                rospy.loginfo("no need to stop at this moment, published -1 as next traffic light info")

    @staticmethod
    def get_traffic_light_state_name(state):
        if state == 0:
            return "RED"
        elif state == 1:
            return "YELLOW"
        elif state == 2:
            return "GREEN"
        else:
            return "UNKNOWN"


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
