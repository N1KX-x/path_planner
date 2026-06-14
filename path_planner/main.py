import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from .occupancy_grid import OccupancyGrid
from .dijkstra_planner import DijkstraPlanner
from .path_follower import PathFollower
from .utils import quaternion_to_yaw, normalize_angle, lidar_range_to_meters

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
    GOAL_TOLERANCE,
    ENABLE_EMERGENCY_STOP,
    RECOVERY_TURN_SPEED,
    RECOVERY_CLEAR_DISTANCE,
    RECOVERY_BACKUP_SPEED,
    RECOVERY_BACKUP_TICKS,
    RECOVERY_MIN_TURN_TICKS
)


class DijkstraNavigator(Node):
    """ROS node that builds a local map, plans with Dijkstra, and drives the robot."""

    def __init__(self):
        """Set up parameters, robot state, ROS topics, and the control timer."""
        super().__init__("dijkstra_navigator")

        # goal_x and goal_y can be overridden with ROS parameters at launch.
        self.declare_parameter("goal_x", DEFAULT_GOAL_X)
        self.declare_parameter("goal_y", DEFAULT_GOAL_Y)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.grid = OccupancyGrid(
            width_m=MAP_WIDTH_M,
            height_m=MAP_HEIGHT_M,
            resolution=GRID_RESOLUTION,
            inflation_radius=OBSTACLE_INFLATION_RADIUS
        )

        self.planner = DijkstraPlanner()
        self.follower = PathFollower()
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
        self.shutdown_requested = False
        self.recovery_mode = False
        self.recovery_state = "BACKUP"
        self.recovery_counter = 0
        self.recovery_turn_direction = 1.0

        self.backup_ticks = RECOVERY_BACKUP_TICKS
        self.min_turn_ticks = RECOVERY_MIN_TURN_TICKS
        self.backup_speed = RECOVERY_BACKUP_SPEED
        self.emergency_stop_active = False
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

        self.estop_sub = self.create_subscription(
            Bool,
            "/emergency_stop",
            self.emergency_stop_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.goal_marker_pub = self.create_publisher(
            MarkerArray,
            "/goal_marker",
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Dijkstra Navigator started.")
        self.get_logger().info(
            f"Target point coordinate: x={self.goal_x:.2f}, y={self.goal_y:.2f} meters"
        )
        self.get_logger().info("Emergency stop topic: /emergency_stop")
        self.get_logger().info("Run keyboard stop in another terminal:")
        self.get_logger().info("ros2 run path_planner emergency_stop_keyboard")

    def emergency_stop_callback(self, msg):
        """Read the /emergency_stop topic and hold the robot when it is active."""
        # The separate keyboard node publishes True/False on this topic.
        self.emergency_stop_active = msg.data

        if self.emergency_stop_active:
            self.get_logger().warn("EMERGENCY STOP ACTIVE")
            self.stop_robot()
        else:
            self.get_logger().info("Emergency stop released")

    def publish_goal_marker(self):
        """Publish a red goal marker in RViz at the current target coordinate."""
        # Delete the old marker first so RViz only shows the current target.
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "goal"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        pin = Marker()
        pin.header.frame_id = "odom"
        pin.header.stamp = self.get_clock().now().to_msg()
        pin.ns = "goal"
        pin.id = 1
        pin.type = Marker.CYLINDER
        pin.action = Marker.ADD
        pin.pose.position.x = self.goal_x
        pin.pose.position.y = self.goal_y
        pin.pose.position.z = 0.25
        pin.pose.orientation.w = 1.0
        pin.scale.x = 0.03
        pin.scale.y = 0.03
        pin.scale.z = 0.50
        pin.color.r = 1.0
        pin.color.g = 0.0
        pin.color.b = 0.0
        pin.color.a = 1.0
        marker_array.markers.append(pin)

        disk = Marker()
        disk.header.frame_id = "odom"
        disk.header.stamp = self.get_clock().now().to_msg()
        disk.ns = "goal"
        disk.id = 2
        disk.type = Marker.CYLINDER
        disk.action = Marker.ADD
        disk.pose.position.x = self.goal_x
        disk.pose.position.y = self.goal_y
        disk.pose.position.z = 0.01
        disk.pose.orientation.w = 1.0
        disk.scale.x = 0.12
        disk.scale.y = 0.12
        disk.scale.z = 0.01
        disk.color.r = 1.0
        disk.color.g = 0.0
        disk.color.b = 0.0
        disk.color.a = 0.7
        marker_array.markers.append(disk)

        text = Marker()
        text.header.frame_id = "odom"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "goal"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.goal_x
        text.pose.position.y = self.goal_y
        text.pose.position.z = 0.75
        text.pose.orientation.w = 1.0
        text.scale.z = 0.20
        text.text = "GOAL"
        text.color.r = 1.0
        text.color.g = 0.0
        text.color.b = 0.0
        text.color.a = 1.0
        marker_array.markers.append(text)

        self.goal_marker_pub.publish(marker_array)

    def odom_callback(self, msg):
        """Convert odometry into the local start frame used by the planner."""
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.initial_pose_set:
            # First odom message becomes the local origin for this run.
            self.initial_x = odom_x
            self.initial_y = odom_y
            self.initial_theta = yaw
            self.initial_pose_set = True
            self.get_logger().info("Initial robot pose saved as origin (0, 0).")
            return

        dx = odom_x - self.initial_x
        dy = odom_y - self.initial_y

        cos_t = math.cos(-self.initial_theta)
        sin_t = math.sin(-self.initial_theta)

        # Rotate odom into the robot's starting frame: +x forward, +y left.
        self.robot_x = dx * cos_t - dy * sin_t
        self.robot_y = dx * sin_t + dy * cos_t
        self.robot_theta = normalize_angle(yaw - self.initial_theta)

    def scan_callback(self, msg):
        """Update the occupancy grid with LiDAR data and start recovery if blocked."""
        self.latest_scan = msg

        if not self.initial_pose_set:
            return

        self.grid.update_from_lidar(
            scan_msg=msg,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta
        )

        if self.obstacle_too_close(msg):
            self.enter_recovery()

    def control_loop(self):
        """Main navigation loop: stop, recover, plan, or follow the next waypoint."""
        if ENABLE_EMERGENCY_STOP and self.emergency_stop_active:
            self.stop_robot()
            return

        if not self.initial_pose_set:
            return

        self.publish_goal_marker()

        if self.goal_reached:
            self.stop_robot()
            return

        if self.recovery_mode:
            self.run_recovery()
            return

        if self.latest_scan is not None and self.obstacle_too_close(self.latest_scan):
            self.enter_recovery()
            self.run_recovery()
            return

        distance_to_goal = math.sqrt(
            (self.goal_x - self.robot_x) ** 2 +
            (self.goal_y - self.robot_y) ** 2
        )

        if distance_to_goal < GOAL_TOLERANCE:
            self.finish_goal()
            return

        if self.need_replan:
            success = self.plan_path()

            if not success:
                self.get_logger().warn("No path found. Robot stopped.")
                self.stop_robot()
                return

            self.need_replan = False

        if not self.path:
            self.stop_robot()
            return

        if self.path_index >= len(self.path):
            final_tolerance = max(GOAL_TOLERANCE, self.follower.waypoint_tolerance)

            if distance_to_goal < final_tolerance:
                self.finish_goal()
                return

            # The last waypoint was reached, but not close enough to the real goal.
            self.get_logger().warn(
                f"Path ended {distance_to_goal:.2f} m from goal. Replanning."
            )
            self.path = []
            self.path_index = 0
            self.need_replan = True
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

        if self.latest_scan is not None and self.obstacle_too_close(self.latest_scan):
            self.enter_recovery()
            return

        self.cmd_pub.publish(cmd)

    def enter_recovery(self):
        """Start recovery mode after the front LiDAR sector becomes unsafe."""
        if self.recovery_mode:
            return

        self.recovery_mode = True
        self.recovery_state = "BACKUP"
        self.recovery_counter = 0
        self.need_replan = True

        # Mark the blocked area before replanning so the next path avoids it.
        self.block_front_danger_zone()

        if self.latest_scan is not None:
            left_dist = self.get_sector_min(self.latest_scan, 45.0, 135.0)
            right_dist = self.get_sector_min(self.latest_scan, -135.0, -45.0)

            if left_dist >= right_dist:
                self.recovery_turn_direction = 1.0
                direction_text = "left"
            else:
                self.recovery_turn_direction = -1.0
                direction_text = "right"

            self.get_logger().warn(
                f"Entering recovery: backing up, then turning {direction_text}. "
                f"Left={left_dist:.2f}, Right={right_dist:.2f}"
            )
        else:
            self.recovery_turn_direction = 1.0
            self.get_logger().warn("Entering recovery: backing up, then turning left.")

    def run_recovery(self):
        """Back up, turn toward clearer space, then allow the planner to replan."""
        cmd = Twist()

        if self.recovery_state == "BACKUP":
            cmd.linear.x = self.backup_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            self.recovery_counter += 1

            if self.recovery_counter >= self.backup_ticks:
                self.recovery_state = "TURN"
                self.recovery_counter = 0
                self.get_logger().warn("Recovery: backup finished, now turning.")

            return

        if self.recovery_state == "TURN":
            self.recovery_counter += 1

            if (
                self.recovery_counter >= self.min_turn_ticks
                and self.latest_scan is not None
                and self.front_is_clear(self.latest_scan)
            ):
                self.get_logger().info("Recovery finished. Front is clear. Replanning.")

                self.stop_robot()

                self.recovery_mode = False
                self.recovery_state = "BACKUP"
                self.recovery_counter = 0

                self.path = []
                self.path_index = 0
                self.need_replan = True

                return

            cmd.linear.x = 0.0
            cmd.angular.z = RECOVERY_TURN_SPEED * self.recovery_turn_direction
            self.cmd_pub.publish(cmd)

            return

    def block_front_danger_zone(self):
        """Mark cells in front of the robot as blocked before planning again."""
        block_distance = 0.90
        half_angle = math.radians(35.0)

        d = 0.15

        while d <= block_distance:
            angle_offset = -half_angle

            while angle_offset <= half_angle:
                world_angle = self.robot_theta + angle_offset

                x = self.robot_x + d * math.cos(world_angle)
                y = self.robot_y + d * math.sin(world_angle)

                cell = self.grid.world_to_grid(x, y)

                if cell is not None:
                    self.grid.inflate_obstacle(cell)

                angle_offset += math.radians(5.0)

            d += self.grid.resolution

    def plan_path(self):
        """Plan a new path from the robot's current cell to the goal cell."""
        start_cell = self.grid.world_to_grid(self.robot_x, self.robot_y)
        requested_goal_cell = self.grid.world_to_grid(self.goal_x, self.goal_y)

        if start_cell is None:
            self.get_logger().warn("Start is outside map.")
            return False

        if requested_goal_cell is None:
            self.get_logger().warn("Goal is outside map.")
            return False

        # Keep the robot's current cell clear so planning can start after recovery.
        self.clear_cell_radius(start_cell, radius_cells=3)

        goal_cell = requested_goal_cell

        if not self.grid.is_free_cell(goal_cell):
            # If the exact goal is occupied, use the closest nearby free cell.
            new_goal_cell = self.find_nearest_free_cell(goal_cell, max_radius_cells=15)

            if new_goal_cell is None:
                self.get_logger().warn("Goal cell is occupied and no nearby free cell was found.")
                return False

            old_goal_world = self.grid.grid_to_world(goal_cell)
            new_goal_world = self.grid.grid_to_world(new_goal_cell)

            self.get_logger().warn(
                f"Goal cell is occupied near ({old_goal_world[0]:.2f}, {old_goal_world[1]:.2f}). "
                f"Using nearby free goal ({new_goal_world[0]:.2f}, {new_goal_world[1]:.2f}) instead."
            )

            goal_cell = new_goal_cell

        cell_path = self.planner.plan(
            grid=self.grid.grid,
            start=start_cell,
            goal=goal_cell
        )

        if cell_path is None or len(cell_path) == 0:
            return False

        world_path = []

        for cell in cell_path:
            x, y = self.grid.grid_to_world(cell)
            world_path.append((x, y))

        step = max(1, PATH_DOWNSAMPLE_STEP)

        self.path = self.downsample_path(
            world_path,
            step=step
        )

        self.path_index = 0

        self.get_logger().info(
            f"New path planned with {len(self.path)} waypoints."
        )

        return True

    def clear_cell_radius(self, center_cell, radius_cells=3):
        """Clear a small circle in the grid so the robot can plan from its own cell."""
        center_row, center_col = center_cell

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                row = center_row + dr
                col = center_col + dc

                if row < 0 or row >= self.grid.rows:
                    continue

                if col < 0 or col >= self.grid.cols:
                    continue

                if math.sqrt(dr ** 2 + dc ** 2) <= radius_cells:
                    self.grid.grid[row][col] = 0

    def find_nearest_free_cell(self, blocked_cell, max_radius_cells=15):
        """Search outward from a blocked goal and return the closest free cell."""
        center_row, center_col = blocked_cell

        if self.grid.is_free_cell(blocked_cell):
            return blocked_cell

        best_cell = None
        best_distance = None

        for radius in range(1, max_radius_cells + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    row = center_row + dr
                    col = center_col + dc

                    if row < 0 or row >= self.grid.rows:
                        continue

                    if col < 0 or col >= self.grid.cols:
                        continue

                    cell = (row, col)

                    if not self.grid.is_free_cell(cell):
                        continue

                    distance = math.sqrt(dr ** 2 + dc ** 2)

                    if best_distance is None or distance < best_distance:
                        best_distance = distance
                        best_cell = cell

            if best_cell is not None:
                return best_cell

        return None

    def downsample_path(self, path, step=1):
        """Keep fewer waypoints so the follower gets a simpler path to track."""
        if len(path) <= 2:
            return path

        step = max(1, step)

        new_path = path[::step]

        if new_path[-1] != path[-1]:
            new_path.append(path[-1])

        return new_path

    def get_sector_min(self, scan_msg, start_deg, end_deg):
        """Return the nearest valid LiDAR reading inside an angle range."""
        values = []

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)

        angle = scan_msg.angle_min

        for raw_r in scan_msg.ranges:
            normalized_angle = math.atan2(math.sin(angle), math.cos(angle))

            in_sector = False

            if start_rad <= end_rad:
                if start_rad <= normalized_angle <= end_rad:
                    in_sector = True
            else:
                if normalized_angle >= start_rad or normalized_angle <= end_rad:
                    in_sector = True

            if in_sector:
                if not math.isinf(raw_r) and not math.isnan(raw_r):
                    r = lidar_range_to_meters(raw_r)

                    if r > 0.0:
                        values.append(r)

            angle += scan_msg.angle_increment

        if len(values) == 0:
            return 999.0

        return min(values)

    def obstacle_too_close(self, scan_msg):
        """Check whether the front sector has an obstacle inside the stop distance."""
        min_front = self.get_sector_min(
            scan_msg,
            -math.degrees(FRONT_DETECTION_ANGLE),
            math.degrees(FRONT_DETECTION_ANGLE)
        )

        if min_front < FRONT_OBSTACLE_STOP_DISTANCE:
            self.get_logger().warn(
                f"Obstacle too close: {min_front:.2f} m"
            )
            return True

        return False

    def front_is_clear(self, scan_msg):
        """Check whether recovery has turned far enough to face open space."""
        min_front = self.get_sector_min(
            scan_msg,
            -math.degrees(FRONT_DETECTION_ANGLE),
            math.degrees(FRONT_DETECTION_ANGLE)
        )

        return min_front > RECOVERY_CLEAR_DISTANCE

    def stop_robot(self):
        """Publish a zero Twist command."""
        if not rclpy.ok():
            return

        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def finish_goal(self):
        """Stop the robot and shut down the node after the target is reached."""
        if self.goal_reached:
            return

        self.get_logger().info("Goal reached!")

        self.goal_reached = True
        self.recovery_mode = False
        self.path = []
        self.path_index = 0
        self.need_replan = False

        for _ in range(10):
            self.stop_robot()

        self.shutdown_requested = True
        self.get_logger().info("Goal reached. Shutting down path planner.")

        if rclpy.ok():
            rclpy.shutdown()

    def stop_for_manual_shutdown(self):
        """Stop the robot when the user exits with Ctrl+C."""
        # Ctrl+C should stop the robot before ROS shuts down the publisher.
        self.emergency_stop_active = True
        self.goal_reached = True
        self.recovery_mode = False
        self.path = []
        self.path_index = 0
        self.need_replan = False

        for _ in range(50):
            self.stop_robot()
            time.sleep(0.03)


def main(args=None):
    """Start the navigator node and stop the robot cleanly on exit."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = DijkstraNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Ctrl+C detected. Sending zero velocity before shutdown...")
    except ExternalShutdownException:
        pass

    finally:
        try:
            node.timer.cancel()
        except Exception:
            pass

        if not node.shutdown_requested:
            node.stop_for_manual_shutdown()
            print("Ctrl+C/manual shutdown: robot stopped at current position.")

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
