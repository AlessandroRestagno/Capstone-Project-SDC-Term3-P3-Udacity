import math
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, max_throttle_percent):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 6.
        ki = 0.001 
        kd = 1.5
        mn = decel_limit #minial throttle value
        mx = max_throttle_percent #maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.05 # 1 / (2pi*tau) = cutoff frequency
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        # return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        filt_current_vel = self.vel_lpf.filt(current_vel)
        #rospy.loginfo('angular_vel: %f', angular_vel)
        #steering = self.yaw_controller.get_steering(twist.twist.linear.x, twist.twist.angular.z, velocity.twist.linear.x)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, filt_current_vel)

        vel_error = linear_vel - filt_current_vel
        
        #rospy.loginfo('linear_vel: %f', linear_vel)
        #rospy.loginfo('filt_current_vel: %f', filt_current_vel)
        #rospy.loginfo('vel_error: %f', vel_error)
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        acceleration = self.throttle_controller.step(vel_error, sample_time)
        #rospy.loginfo('angular_vel: %.3f   linear_vel: %.3f   filt_current_vel: %.3f   vel_error: %.3f  acceleration: %.3f', angular_vel, linear_vel, filt_current_vel, vel_error, acceleration)
        
        throttle = acceleration
        brake = 0.

        if linear_vel == 0. and filt_current_vel < 0.1:
            throttle = 0.
            brake = 700. # N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif acceleration <= 0.05 and vel_error < 0.:
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m

        return throttle, brake, steering
