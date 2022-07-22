from math import pi
from enum import Enum


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:

    def __init__(self):
        # application parameters
        self.show_animation = True

        # prediction parameters
        self.dt = 1.0  # [s] Time tick for motion prediction
        self.predict_time = 1.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 10.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_bump_cost_gain = 100

        # robot parameters
        self.max_speed = 0.2  # [m/s]
        self.min_speed = -0.2  # [m/s]
        self.max_yaw_rate = 20.0 * pi / 180.0  # [rad/s]
        self.max_accel = 0.4  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 1.0 * pi / 180.0  # [rad/s]
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.3  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.4 + 0.1  # [m] for collision check
        self.robot_length = 0.45 * 2 + 0.1  # [m] for collision check
