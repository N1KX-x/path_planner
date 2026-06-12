# 26summer-ROS-robot
# TurtleBot3 Dijkstra Navigation with LiDAR

This project implements a basic autonomous navigation system for TurtleBot3 using **Dijkstra's algorithm** and **LiDAR-based obstacle detection**.

The robot starts from its current position and treats that position as the origin `(0, 0)`. The user gives a target coordinate, and the robot plans a path from the origin to the goal while avoiding obstacles detected by the TurtleBot's LiDAR.

---

## Project Goal

The goal of this project is to make a TurtleBot3 move from one point to another using:

* Dijkstra's shortest path algorithm
* LiDAR obstacle detection
* Odometry-based position tracking
* ROS 2 velocity commands
* A local occupancy grid map

The robot should be able to:

1. Start at an unknown location.
2. Treat the starting position as `(0, 0)`.
3. Receive a final goal coordinate.
4. Detect obstacles using LiDAR.
5. Build a grid map of free and occupied space.
6. Use Dijkstra's algorithm to find a safe path.
7. Follow the path using `/cmd_vel`.
8. Stop and replan if an obstacle is detected too close in front.

---

## System Overview

The navigation system has three main parts:

### 1. Occupancy Grid Mapping

The environment is represented as a 2D grid.

Each grid cell can be:

```text
0 = free space
1 = obstacle
```

The LiDAR scan is used to mark obstacle cells in the grid.

Obstacle inflation is also used, meaning cells around an obstacle are marked as blocked too. This gives the robot extra safety space.

---

### 2. Dijkstra Path Planning

Dijkstra's algorithm searches the occupancy grid and finds the shortest collision-free path from the robot's current position to the goal.

The planner avoids cells marked as obstacles.

---

### 3. Path Following

After the path is generated, the robot follows the path waypoint by waypoint.

The path follower calculates:

* Distance to the next waypoint
* Angle error between the robot and the waypoint
* Forward velocity
* Turning velocity

The robot publishes velocity commands to:

```text
/cmd_vel
```

---

## ROS 2 Topics Used

This project uses the following TurtleBot3 ROS 2 topics:

| Topic      | Type                        | Purpose                 |
| ---------- | --------------------------- | ----------------------- |
| `/scan`    | `sensor_msgs/msg/LaserScan` | Reads LiDAR data        |
| `/odom`    | `nav_msgs/msg/Odometry`     | Tracks robot position   |
| `/cmd_vel` | `geometry_msgs/msg/Twist`   | Sends movement commands |

---

## Package Structure

```text
turtlebot3_dijkstra_nav/
├── package.xml
├── setup.py
├── resource/
│   └── turtlebot3_dijkstra_nav
├── test/
└── turtlebot3_dijkstra_nav/
    ├── __init__.py
    ├── main_node.py
    ├── dijkstra_planner.py
    ├── occupancy_grid.py
    ├── path_follower.py
    ├── utils.py
    └── config.py
```

---

## File Descriptions

### `main_node.py`

This is the main ROS 2 node.

It connects all parts of the program together.

Responsibilities:

* Subscribe to `/scan`
* Subscribe to `/odom`
* Publish to `/cmd_vel`
* Store the goal coordinate
* Update the map using LiDAR
* Run Dijkstra path planning
* Follow the planned path
* Stop and replan when obstacles are too close

---

### `dijkstra_planner.py`

This file contains the Dijkstra path planning algorithm.

Responsibilities:

* Take the occupancy grid
* Take the start cell
* Take the goal cell
* Search for the shortest path
* Return a list of grid cells representing the path

---

### `occupancy_grid.py`

This file manages the map.

Responsibilities:

* Create a 2D occupancy grid
* Convert world coordinates to grid coordinates
* Convert grid coordinates to world coordinates
* Mark obstacles from LiDAR readings
* Inflate obstacles for safety

---

### `path_follower.py`

This file controls how the robot follows the path.

Responsibilities:

* Calculate distance to the next waypoint
* Calculate angle error
* Rotate toward the waypoint
* Move forward when aligned
* Generate `/cmd_vel` velocity commands

---

### `utils.py`

This file contains helper math functions.

Responsibilities:

* Normalize angles
* Convert quaternion orientation to yaw
* Calculate distance between points
* Clamp speed values

---

### `config.py`

This file contains all tuning values.

Examples:

* Default goal coordinate
* Map size
* Grid resolution
* Obstacle inflation radius
* Maximum robot speed
* Turning speed
* Waypoint tolerance
* Emergency stop distance

This makes it easier to tune the robot without editing the main program logic.

---

## Installation

Go to your ROS 2 workspace:

```bash
cd ~/turtlebot3_ws/src
```

Create the ROS 2 Python package:

```bash
ros2 pkg create turtlebot3_dijkstra_nav --build-type ament_python --dependencies rclpy sensor_msgs nav_msgs geometry_msgs
```

Place the Python files inside:

```bash
~/turtlebot3_ws/src/turtlebot3_dijkstra_nav/turtlebot3_dijkstra_nav/
```

The Python module folder should contain:

```text
__init__.py
main_node.py
dijkstra_planner.py
occupancy_grid.py
path_follower.py
utils.py
config.py
```

---

## Setup.py Entry Point

In `setup.py`, add the console script:

```python
entry_points={
    'console_scripts': [
        'main_node = turtlebot3_dijkstra_nav.main_node:main',
    ],
},
```

This allows the node to be run using:

```bash
ros2 run turtlebot3_dijkstra_nav main_node
```

---

## Build the Package

From the workspace root:

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_dijkstra_nav
source install/setup.bash
```

---

## Running the Program

### Run with Default Goal

The default goal is set in `config.py`.

```bash
ros2 run turtlebot3_dijkstra_nav main_node
```

---

### Run with Custom Goal

Example:

```bash
ros2 run turtlebot3_dijkstra_nav main_node --ros-args -p goal_x:=2.0 -p goal_y:=1.5
```

This means:

```text
goal_x = 2.0 meters forward
goal_y = 1.5 meters left
```

The robot's starting position is treated as:

```text
(0, 0)
```

---

## Normal TurtleBot3 Run Order

In Terminal 1, start the TurtleBot3 bringup:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

In Terminal 2, run the Dijkstra navigation node:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_dijkstra_nav main_node --ros-args -p goal_x:=2.0 -p goal_y:=1.0
```

---

## Check Required Topics

Before running the navigation program, check that the required topics exist:

```bash
ros2 topic list
```

You should see:

```text
/scan
/odom
/cmd_vel
```

If `/scan` is missing, the LiDAR is not running.

If `/odom` is missing, odometry is not running.

If `/cmd_vel` is missing, the robot is not ready to receive velocity commands.

---

## Configuration

All tuning values are located in:

```text
config.py
```

Important values:

```python
DEFAULT_GOAL_X = 2.0
DEFAULT_GOAL_Y = 1.0

MAP_WIDTH_M = 6.0
MAP_HEIGHT_M = 6.0
GRID_RESOLUTION = 0.1

OBSTACLE_INFLATION_RADIUS = 0.20
MAX_LIDAR_OBSTACLE_RANGE = 3.0
FRONT_OBSTACLE_STOP_DISTANCE = 0.30

MAX_LINEAR_SPEED = 0.15
MAX_ANGULAR_SPEED = 0.8
WAYPOINT_TOLERANCE = 0.12
K_LINEAR = 0.6
K_ANGULAR = 1.5
```

For first testing, it is safer to use slower speed:

```python
MAX_LINEAR_SPEED = 0.10
MAX_ANGULAR_SPEED = 0.6
```

---

## Algorithm Flow

```text
1. Start the robot.
2. Save the robot's initial odometry pose.
3. Treat the initial pose as origin (0, 0).
4. Read the goal coordinate.
5. Use LiDAR to mark obstacles in the occupancy grid.
6. Convert the robot position and goal position into grid cells.
7. Run Dijkstra's algorithm.
8. Convert the grid path into world waypoints.
9. Follow each waypoint using velocity control.
10. If an obstacle is too close, stop and replan.
11. Stop when the robot reaches the goal.
```




## Project Summary

This project demonstrates how a TurtleBot3 can navigate from a starting point to a target coordinate using Dijkstra's algorithm and LiDAR obstacle detection.

The robot treats its starting position as the origin, builds a local occupancy grid from LiDAR data, plans a collision-free path using Dijkstra, and follows the path using ROS 2 velocity commands.
