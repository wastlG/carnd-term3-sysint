from lowpass import LowPassFilter
from pid import PID
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
        self.lowpass = LowPassFilter(self.accel_limit, self.delta_t)
        self.pid = PID(2.0, 0.4, 0.1, mn=-0.2, mx=0.2)

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
            steer = target_angular_velocity * self.steer_ratio
            velocity_error = target_linear_velocity - current_linear_velocity
            throttle = self.pid.step(velocity_error, self.delta_t)
            if throttle < 0.:
                brake = -throttle
                throttle = 0.
        else:
            self.pid.reset()
        
        return throttle, brake, steer
