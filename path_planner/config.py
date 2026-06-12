# Default goal coordinate
DEFAULT_GOAL_X = 2.0
DEFAULT_GOAL_Y = 1.0

# Occupancy grid map settings (Map size in meters)
MAP_WIDTH_M = 15.0
MAP_HEIGHT_M = 15.0

# Each grid cell size in meters.
# Smaller value = more accurate but slower.
# 0.1 means each cell is 10 cm.
GRID_RESOLUTION = 0.1

# Safety radius around obstacles.
# If obstacle is detected, nearby cells are also marked as blocked.
OBSTACLE_INFLATION_RADIUS = 0.20

# LiDAR obstacle settings

# Ignore LiDAR obstacles farther than this distance.
MAX_LIDAR_OBSTACLE_RANGE = 3.0

# Emergency stop distance in front of robot.
FRONT_OBSTACLE_STOP_DISTANCE = 0.30

# Front detection angle range in radians.
# 0.35 rad is about 20 degrees.
FRONT_DETECTION_ANGLE = 0.35

# Path following settings

# Maximum forward speed.
MAX_LINEAR_SPEED = 0.15

# Maximum turning speed.
MAX_ANGULAR_SPEED = 0.8

# Distance from waypoint that counts as "reached".
WAYPOINT_TOLERANCE = 0.12

# Forward speed gain.
K_LINEAR = 0.6

# Turning speed gain.
K_ANGULAR = 1.5

# If angle error is larger than this,
# robot rotates first instead of moving forward.
ROTATE_FIRST_ANGLE = 0.35

# Keep every N-th grid path point as waypoint.
# Larger value = fewer waypoints, smoother but less precise.
PATH_DOWNSAMPLE_STEP = 3

# Robot stops when it is this close to the final goal.
GOAL_TOLERANCE = 0.1