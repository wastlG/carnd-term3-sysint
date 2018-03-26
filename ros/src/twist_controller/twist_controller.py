from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.delta_t = 1. / 50. #asdasdf
        self.vehicle_mass = args[0]
        self.brake_deadband = args[1]
        self.decel_limit = args[2]
        self.accel_limit = args[3]
        self.wheel_radius = args[4]
        self.wheel_base = args[5]
        self.steer_ratio = args[6]
        self.max_steer_angle = args[7]
        self.max_lat_accel = args[8]
        self.lowpass = LowPassFilter(self.accel_limit, self.delta_t)
        #self.pid = PID(2.0, 0.4, 0.1, mn=-0.2, mx=0.2)
        self.pid = PID(1.2, .2, 0.1, mn=args[2], mx=args[3])        
        self.steering_controller = YawController(args[5], args[6], 0.0, args[8], args[7])


    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_linear_velocity = args[0]
        target_angular_velocity = args[1]
        current_linear_velocity = args[2]
        dbw_enabled = args[3]
        
        throttle = 0.
        brake = 0.
        steer = 0.
        
        if dbw_enabled:
            #steer = target_angular_velocity * self.steer_ratio
            steer = self.steering_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
            velocity_error = target_linear_velocity - current_linear_velocity
            throttle = self.pid.step(velocity_error, self.delta_t) # [m/s^2]
            if throttle < 0.:
                #brake = -throttle
                # maybe its better to calculate the deceleration as "brake torque"
                brake = self.vehicle_mass * abs(throttle) * self.wheel_radius # [Nm]
                throttle = 0.
        else:
            self.pid.reset()
        
        return throttle, brake, steer
