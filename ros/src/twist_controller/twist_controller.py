from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
THROTTLE_KP = 0.7
THROTTLE_KI = 0.0007
THROTTLE_KD = 0.1


class TwistController(object):
    def __init__(self, *args, **kwargs):
        self.wheel_base = kwargs['wheel_base']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        self.min_speed = 0.
        self.max_lat_accel = kwargs['max_lat_accel']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.brake_deadband = kwargs['brake_deadband']
        self.max_throttle_percentage = kwargs['max_throttle_percentage']
        self.max_brake_percentage = kwargs['max_brake_percentage']
        self.torque = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        # Lateral control
        self.lat_controller = YawController(
            wheel_base=self.wheel_base, steer_ratio=kwargs['steer_ratio'],
            min_speed=self.min_speed, max_lat_accel=self.max_lat_accel, max_steer_angle=self.max_steer_angle)
        self.steer_lpf = LowPassFilter(tau=150, ts=75)

        # Longitudinal control
        self.lon_controller = PID(
            kp=THROTTLE_KP, ki=THROTTLE_KI, kd=THROTTLE_KD,
            mn=self.decel_limit, mx=self.accel_limit)
        self.throttle_lpf = LowPassFilter(tau=150, ts=75)
        self.linear_vel_lpf = LowPassFilter(tau=150, ts=75)

    def control(self, proposed_twist, current_twist, dbw_enabled):
        proposed_linear_vel = abs(proposed_twist.linear.x)
        proposed_angular_vel = proposed_twist.angular.z

        current_linear_vel = current_twist.linear.x
        current_linear_vel = self.linear_vel_lpf.filt(current_linear_vel)

        delta_linear_vel = proposed_linear_vel - current_linear_vel

        if dbw_enabled:
            # Get throttle via pid controller
            throttle = self.lon_controller.step(delta_linear_vel, 0.02)
            throttle = min(throttle, self.max_throttle_percentage)
            throttle = self.throttle_lpf.filt(throttle)

            # Get brake through torque calculation, if throttle < 0
            brake = 0.0
            if throttle < 0.0:
                if -throttle > self.brake_deadband:
                    brake = -self.torque * throttle
                throttle = 0.0

            # Get steer via yaw_controller
            steer = self.lat_controller.get_steering(proposed_linear_vel, proposed_angular_vel, current_linear_vel)
            steer = self.steer_lpf.filt(steer)
            steer = max(-self.max_steer_angle, min(steer, self.max_steer_angle))
        else:
            throttle = 0.
            brake = 0.
            steer = 0.
            self.steer_lpf.reset()
            self.linear_vel_lpf.reset()
            self.lon_controller.reset()

        return throttle, brake, steer
