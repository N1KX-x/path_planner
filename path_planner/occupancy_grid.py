import math

from .config import MAX_LIDAR_OBSTACLE_RANGE


class OccupancyGrid:
    def __init__(self, width_m=6.0, height_m=6.0, resolution=0.1, inflation_radius=0.2):
        """
        width_m: map width in meters
        height_m: map height in meters
        resolution: meters per grid cell
        inflation_radius: extra safety distance around obstacles
        """

        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution
        self.inflation_radius = inflation_radius

        self.cols = int(width_m / resolution)
        self.rows = int(height_m / resolution)

        # Robot origin is placed at the center of the grid
        self.origin_row = self.rows // 2
        self.origin_col = self.cols // 2

        # 0 = free, 1 = obstacle
        self.grid = []

        for _ in range(self.rows):
            row = [0 for _ in range(self.cols)]
            self.grid.append(row)

    # --------------------------------------------------
    # Convert world coordinate to grid coordinate
    # --------------------------------------------------
    def world_to_grid(self, x, y):
        """
        World:
            x forward/backward in meters
            y left/right in meters

        Grid:
            row, col
        """

        col = int(round(x / self.resolution)) + self.origin_col
        row = self.origin_row - int(round(y / self.resolution))

        if row < 0 or row >= self.rows:
            return None

        if col < 0 or col >= self.cols:
            return None

        return (row, col)

    # --------------------------------------------------
    # Convert grid coordinate to world coordinate
    # --------------------------------------------------
    def grid_to_world(self, cell):
        row, col = cell

        x = (col - self.origin_col) * self.resolution
        y = (self.origin_row - row) * self.resolution

        return x, y

    # --------------------------------------------------
    # Check if cell is free
    # --------------------------------------------------
    def is_free_cell(self, cell):
        row, col = cell

        if row < 0 or row >= self.rows:
            return False

        if col < 0 or col >= self.cols:
            return False

        return self.grid[row][col] == 0

    # --------------------------------------------------
    # Mark obstacle
    # --------------------------------------------------
    def mark_obstacle_world(self, x, y):
        cell = self.world_to_grid(x, y)

        if cell is None:
            return

        self.inflate_obstacle(cell)

    # --------------------------------------------------
    # Inflate obstacle for safety
    # --------------------------------------------------
    def inflate_obstacle(self, center_cell):
        center_row, center_col = center_cell

        inflation_cells = int(self.inflation_radius / self.resolution)

        for dr in range(-inflation_cells, inflation_cells + 1):
            for dc in range(-inflation_cells, inflation_cells + 1):
                row = center_row + dr
                col = center_col + dc

                if row < 0 or row >= self.rows:
                    continue

                if col < 0 or col >= self.cols:
                    continue

                distance = math.sqrt(dr ** 2 + dc ** 2) * self.resolution

                if distance <= self.inflation_radius:
                    self.grid[row][col] = 1

    # --------------------------------------------------
    # Update grid from LiDAR
    # --------------------------------------------------
    def update_from_lidar(self, scan_msg, robot_x, robot_y, robot_theta):
        """
        Converts LiDAR readings into obstacle positions.
        """

        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan_msg.angle_increment
                continue

            if r < scan_msg.range_min or r > scan_msg.range_max:
                angle += scan_msg.angle_increment
                continue

            # Ignore far readings
            if r > MAX_LIDAR_OBSTACLE_RANGE:
                angle += scan_msg.angle_increment
                continue

            world_angle = robot_theta + angle

            obstacle_x = robot_x + r * math.cos(world_angle)
            obstacle_y = robot_y + r * math.sin(world_angle)

            self.mark_obstacle_world(obstacle_x, obstacle_y)

            angle += scan_msg.angle_increment

    # --------------------------------------------------
    # Optional: clear map
    # --------------------------------------------------
    def clear(self):
        for r in range(self.rows):
            for c in range(self.cols):
                self.grid[r][c] = 0