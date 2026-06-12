import math

from geometry_msgs.msg import Twist

from .utils import normalize_angle, clamp

from .config import (
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    WAYPOINT_TOLERANCE,
    K_LINEAR,
    K_ANGULAR,
    ROTATE_FIRST_ANGLE
)


class PathFollower:
    def __init__(self):
        self.max_linear_speed = MAX_LINEAR_SPEED
        self.max_angular_speed = MAX_ANGULAR_SPEED

        self.waypoint_tolerance = WAYPOINT_TOLERANCE

        self.k_linear = K_LINEAR
        self.k_angular = K_ANGULAR

        self.rotate_first_angle = ROTATE_FIRST_ANGLE

    def compute_cmd_vel(self, robot_x, robot_y, robot_theta, target_x, target_y):
        """
        Takes robot pose and target point.

        Returns:
            cmd_vel message
            reached_waypoint boolean
        """

        cmd = Twist()

        dx = target_x - robot_x
        dy = target_y - robot_y

        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < self.waypoint_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd, True

        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - robot_theta)

        # If robot is not facing the waypoint, rotate first
        if abs(angle_error) > self.rotate_first_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = self.k_angular * angle_error
            cmd.angular.z = clamp(
                cmd.angular.z,
                -self.max_angular_speed,
                self.max_angular_speed
            )
            return cmd, False

        # Otherwise move forward and slightly correct angle
        linear_speed = self.k_linear * distance
        angular_speed = self.k_angular * angle_error

        cmd.linear.x = clamp(
            linear_speed,
            0.04,
            self.max_linear_speed
        )

        cmd.angular.z = clamp(
            angular_speed,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        return cmd, False