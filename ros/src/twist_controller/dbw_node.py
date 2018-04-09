#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import TwistController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

LOG_LEVEL = rospy.INFO

# Adjust execution rate to reduce latency between vm and simulator on host. Udacity default=50
DBW_CONTROLLER_RATE = 25


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=LOG_LEVEL)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Fetch value from parameter server
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        max_throttle_percentage = rospy.get_param('~max_throttle_percentage', 0.8)
        max_brake_percentage = rospy.get_param('~max_brake_percentage', -0.8)

        params = {
            'vehicle_mass': vehicle_mass,
            'fuel_capacity': fuel_capacity,
            'brake_deadband': brake_deadband,
            'decel_limit': decel_limit,
            'accel_limit': accel_limit,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'steer_ratio': steer_ratio,
            'max_lat_accel': max_lat_accel,
            'max_steer_angle': max_steer_angle,
            'max_throttle_percentage': max_throttle_percentage,
            'max_brake_percentage': max_brake_percentage
        }

        self.current_velocity = None
        self.twist_cmd = None
        self.dbw_enabled = False
        self.twist_controller = TwistController(**params)

        self.loop()

    def loop(self):
        rate = rospy.Rate(DBW_CONTROLLER_RATE)
        while not rospy.is_shutdown():
            if self.current_velocity and self.twist_cmd:
                # Run twist controller to get throttle brake and steer
                throttle, brake, steer = self.twist_controller.control(
                    proposed_twist=self.twist_cmd.twist,
                    current_twist=self.current_velocity.twist,
                    dbw_enabled=self.dbw_enabled)
                rospy.loginfo("throttle=%.3f, brake=%.3f, steer=%.3f", throttle, brake, steer)

                # Publish only if dbw_enabled
                if self.dbw_enabled:
                    self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg
        # rospy.loginfo(msg)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg


if __name__ == '__main__':
    DBWNode()
