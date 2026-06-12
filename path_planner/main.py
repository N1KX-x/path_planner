import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Boll
import math

from .occupancy_grid import OccupancyGrid
from .dijkstra_planner import DijkstraPlanner
from .path_follower import PathFollower
from .utils import quaternion_to_yaw, normalize_angle

from .config import (
    DEFAULT_GOAL_X,
    DEFAULT_GOAL_Y,
    MAP_WIDTH_M,
    MAP_HEIGHT_M,
    GRID_RESOLUTION,
    OBSTACLE_INFLATION_RADIUS,
    FRONT_OBSTACLE_STOP_DISTANCE,
    FRONT_DETECTION_ANGLE,
    PATH_DOWNSAMPLE_STEP,
    GOAL_TOLERANCE
)


class DijkstraNavigator(Node):
    def __init__(self):
        super().__init__("dijkstra_navigator")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("goal_x", DEFAULT_GOAL_X)
        self.declare_parameter("goal_y", DEFAULT_GOAL_Y)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        # -----------------------------
        # Map, planner, follower
        # -----------------------------
        self.grid = OccupancyGrid(
            width_m=MAP_WIDTH_M,
            height_m=MAP_HEIGHT_M,
            resolution=GRID_RESOLUTION,
            inflation_radius=OBSTACLE_INFLATION_RADIUS
        )

        self.planner = DijkstraPlanner()
        self.follower = PathFollower()

        # -----------------------------
        # Robot state
        # -----------------------------
        self.initial_pose_set = False

        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.latest_scan = None

        self.path = []
        self.path_index = 0
        self.need_replan = True

        self.goal_reached = False

        # -----------------------------
        # ROS publishers/subscribers
        # -----------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Dijkstra Navigator started.")
        self.get_logger().info(f"Goal set to: ({self.goal_x}, {self.goal_y})")

    # --------------------------------------------------
    # Odometry callback
    # --------------------------------------------------
    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # Set the robot's starting position as origin
        if not self.initial_pose_set:
            self.initial_x = odom_x
            self.initial_y = odom_y
            self.initial_theta = yaw
            self.initial_pose_set = True

            self.get_logger().info("Initial robot pose saved as origin (0, 0).")
            return

        # Convert odom pose into local coordinate frame
        dx = odom_x - self.initial_x
        dy = odom_y - self.initial_y

        # Rotate into the robot-start coordinate system
        cos_t = math.cos(-self.initial_theta)
        sin_t = math.sin(-self.initial_theta)

        self.robot_x = dx * cos_t - dy * sin_t
        self.robot_y = dx * sin_t + dy * cos_t
        self.robot_theta = normalize_angle(yaw - self.initial_theta)

    # --------------------------------------------------
    # LiDAR callback
    # --------------------------------------------------
    def scan_callback(self, msg):
        self.latest_scan = msg

        if not self.initial_pose_set:
            return

        # Update obstacle map from LiDAR
        self.grid.update_from_lidar(
            scan_msg=msg,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta
        )

        # If something is too close in front, force replanning
        if self.obstacle_too_close(msg):
            self.stop_robot()
            self.need_replan = True

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------
    def control_loop(self):
        if not self.initial_pose_set:
            return

        if self.goal_reached:
            self.stop_robot()
            return

        # Check if goal reached
        distance_to_goal = math.sqrt(
            (self.goal_x - self.robot_x) ** 2 +
            (self.goal_y - self.robot_y) ** 2
        )

        if distance_to_goal < GOAL_TOLERANCE:
            self.get_logger().info("Goal reached!")
            self.goal_reached = True
            self.stop_robot()
            return

        # Replan if needed
        if self.need_replan:
            success = self.plan_path()

            if not success:
                self.get_logger().warn("No path found. Robot stopped.")
                self.stop_robot()
                return

            self.need_replan = False

        # If path is empty, stop
        if not self.path:
            self.stop_robot()
            return

        # Follow path
        if self.path_index >= len(self.path):
            self.stop_robot()
            return

        target = self.path[self.path_index]

        cmd, reached_waypoint = self.follower.compute_cmd_vel(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta,
            target_x=target[0],
            target_y=target[1]
        )

        if reached_waypoint:
            self.path_index += 1
            return

        self.cmd_pub.publish(cmd)

    # --------------------------------------------------
    # Dijkstra planning
    # --------------------------------------------------
    def plan_path(self):
        start_cell = self.grid.world_to_grid(self.robot_x, self.robot_y)
        goal_cell = self.grid.world_to_grid(self.goal_x, self.goal_y)

        if start_cell is None:
            self.get_logger().warn("Start is outside map.")
            return False

        if goal_cell is None:
            self.get_logger().warn("Goal is outside map.")
            return False

        if not self.grid.is_free_cell(goal_cell):
            self.get_logger().warn("Goal cell is occupied.")
            return False

        cell_path = self.planner.plan(
            grid=self.grid.grid,
            start=start_cell,
            goal=goal_cell
        )

        if cell_path is None or len(cell_path) == 0:
            return False

        # Convert grid path to world path
        world_path = []
        for cell in cell_path:
            x, y = self.grid.grid_to_world(cell)
            world_path.append((x, y))

        # Reduce number of waypoints
        self.path = self.downsample_path(world_path, step=PATH_DOWNSAMPLE_STEP)
        self.path_index = 0

        self.get_logger().info(f"New path planned with {len(self.path)} waypoints.")

        return True

    # --------------------------------------------------
    # Reduce path points
    # --------------------------------------------------
    def downsample_path(self, path, step=3):
        if len(path) <= 2:
            return path

        new_path = path[::step]

        if new_path[-1] != path[-1]:
            new_path.append(path[-1])

        return new_path

    # --------------------------------------------------
    # Emergency obstacle detection
    # --------------------------------------------------
    def obstacle_too_close(self, scan_msg):
        front_ranges = []

        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan_msg.angle_increment
                continue

            # Front area
            if -FRONT_DETECTION_ANGLE <= angle <= FRONT_DETECTION_ANGLE:
                front_ranges.append(r)

            angle += scan_msg.angle_increment

        if len(front_ranges) == 0:
            return False

        min_front = min(front_ranges)

        if min_front < FRONT_OBSTACLE_STOP_DISTANCE:
            return True

        return False

    # --------------------------------------------------
    # Stop robot
    # --------------------------------------------------
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = DijkstraNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()