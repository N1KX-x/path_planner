# How to Run
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py

ros2 run turtlebot3_dijkstra_nav main_node --ros-args -p goal_x:=2.0 -p goal_y:=1.5

