import heapq
import math
class DijkstraPlanner:
    def __init__(self):
        pass

    def plan(self, grid, start, goal):
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

        cost_so_far = {}
        came_from = {}

        cost_so_far[start] = 0
        came_from[start] = None

        while priority_queue:
            current_cost, current = heapq.heappop(priority_queue)

            if current == goal:
                break

            for neighbor, move_cost in self.get_neighbors(current, grid):
                new_cost = cost_so_far[current] + move_cost

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(priority_queue, (new_cost, neighbor))

        if goal not in came_from:
            return None

        path = self.reconstruct_path(came_from, start, goal)

        return path

    def get_neighbors(self, cell, grid):
        row, col = cell

        directions = [
            (-1, 0, 1.0),     # up
            (1, 0, 1.0),      # down
            (0, -1, 1.0),     # left
            (0, 1, 1.0),      # right

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

            neighbors.append(((nr, nc), move_cost))

        return neighbors

    def in_bounds(self, cell, rows, cols):
        row, col = cell

        if row < 0 or row >= rows:
            return False

        if col < 0 or col >= cols:
            return False

        return True

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []

        while current is not None:
            path.append(current)
            current = came_from[current]

        path.reverse()

        if path[0] != start:
            return None

        return path