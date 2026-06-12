import math


def normalize_angle(angle):
    """
    Converts angle to range [-pi, pi].
    """

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw


def distance_between_points(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1

    return math.sqrt(dx ** 2 + dy ** 2)


def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value