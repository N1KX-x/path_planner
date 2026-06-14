import heapq
import math


class DijkstraPlanner:
    """Find the lowest-cost path through a 2D occupancy grid."""

    def plan(self, grid, start, goal):
        """Run Dijkstra from start cell to goal cell and return the cell path."""
        rows = len(grid)
        cols = len(grid[0])

        if not self.in_bounds(start, rows, cols):
            return None

        if not self.in_bounds(goal, rows, cols):
            return None

        if grid[start[0]][start[1]] == 1:
            return None

        if grid[goal[0]][goal[1]] == 1:
            return None

        priority_queue = []
        heapq.heappush(priority_queue, (0, start))

        # cost_so_far stores the cheapest known distance from start to each cell.
        cost_so_far = {start: 0}
        came_from = {start: None}

        while priority_queue:
            current_cost, current = heapq.heappop(priority_queue)

            if current == goal:
                break

            for neighbor, move_cost in self.get_neighbors(current, grid):
                new_cost = current_cost + move_cost

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(priority_queue, (new_cost, neighbor))

        if goal not in came_from:
            return None

        return self.reconstruct_path(came_from, start, goal)

    def get_neighbors(self, cell, grid):
        """Return free neighbor cells and their movement costs."""
        row, col = cell

        directions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        ]

        rows = len(grid)
        cols = len(grid[0])
        neighbors = []

        for dr, dc, move_cost in directions:
            nr = row + dr
            nc = col + dc

            if not self.in_bounds((nr, nc), rows, cols):
                continue

            if grid[nr][nc] == 1:
                continue

            # Do not let a diagonal move squeeze through the corner of an obstacle.
            if dr != 0 and dc != 0:
                if grid[row + dr][col] == 1:
                    continue

                if grid[row][col + dc] == 1:
                    continue

            neighbors.append(((nr, nc), move_cost))

        return neighbors

    def in_bounds(self, cell, rows, cols):
        """Check whether a row and column are inside the grid."""
        row, col = cell
        return 0 <= row < rows and 0 <= col < cols

    def reconstruct_path(self, came_from, start, goal):
        """Walk backward from the goal to rebuild the final path."""
        current = goal
        path = []

        while current is not None:
            path.append(current)
            current = came_from[current]

        path.reverse()

        if path[0] != start:
            return None

        return path
