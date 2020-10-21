import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(
        self,
        vehicle_mass,
        fuel_capacity,
        brake_deadband,
        decel_limit,
        accel_limit,
        wheel_radius,
        wheel_base,
        steer_ratio,
        max_lat_accel,
        max_steer_angle,
    ):
        # Init yaw controller
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle
        )

        # PID Params - determined experimentally
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  # Min throttle value
        mx = 0.2  # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # LP filter params
        # velocity that's coming in over the messages is noisy
        # LP filter in this case is filtering all high frequency noise in velocity
        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        # Other infos
        self.vehice_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # if manual control is enabled, don't do anything
        if not dbw_enabled:
            # avoid error accumulation
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # filter current velocity
        current_vel = self.vel_lpf.filt(current_vel)

        # calculate steering angle
        steering = self.yaw_controller.get_steering(
            linear_vel, angular_vel, current_vel
        )

        # check what is current vel error from target vel
        vel_error = linear_vel - current_vel
        # update last vel
        self.last_vel = current_vel

        # calculate time step (needed for PID)
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # use PID to calculate throttle
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # if target velocity is 0 and current velocity is very small that means we need to stop
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            # apply brake - to hold the car in place if we are stopping at a light
            brake = 400  # torque [N * m]
        # if throttle is small and we are going over target velocity
        elif throttle < 0.1 and vel_error < 0:
            # let go of throttle
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            # apply brake - slightly
            brake = abs(decel) * self.vehice_mass * self.wheel_radius  # torque [N * m]

        return throttle, brake, steering
