# Autonomous-Frontier-Exploration-with-RRT-in-ROS-2
This project implements Autonomous Frontier Exploration on a TurtleBot3 robot in ROS 2 Humble using a Rapidly-exploring Random Tree (RRT) planner.

## The robot autonomously explores unknown environments by:

Building a map with SLAM (e.g., slam_toolbox or gmapping).
Detecting unexplored frontiers in the occupancy grid.
Using RRT to plan paths to reachable frontiers.
Sending navigation goals to Nav2 until the environment is fully explored.
Returning to its start position once exploration is complete.
This package works in both simulation (Gazebo) and on a real TurtleBot3.

## 📂 Project Structure:
```
autonomous_explorer/
├── launch/
│ ├── autonomous_explorer.launch.py # Main launch file
│ └── turtlebot3_world.launch.py # Example world launch file
├── autonomous_explorer/
│ ├── init.py
│ ├── frontier_explorer.py # Main exploration node
│ ├── frontier_detector.py # Detects frontiers in the occupancy grid
│ ├── rrt_planner.py # RRT-based path planner
├── package.xml
├── setup.py
└── README.md
```

---

## 🛠️ Installation

### 1. Install Nav2 and TurtleBot3
```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
```
### 2. Install SLAM Toolbox
```
sudo apt install ros-humble-slam-toolbox
```
### 3. Clone the repository in your ROS 2 workspace
```
cd ~/ros2_ws/src
git clone https://github.com/Ureed-Hussain/Autonomous-Frontier-Exploration-with-RRT-in-ROS-2.git autonomous_explorer
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## 🌍 Running in Simulation
![W1-MadewithClipchamp-ezgif com-video-to-gif-converter(2)](https://github.com/user-attachments/assets/e513dc62-b57e-48cc-aaf0-fa543ca65ef4)

### 1. Start the TurtleBot3 world in Gazebo

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
### 2. Start the Navigation 2 stack

(Use use_sim_time:=True in Gazebo, skip it on the real robot)
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
### 3. Start SLAM Toolbox
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
### 4. Start RViz for visualization
```
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
### 5. Run the autonomous explorer node
```
ros2 run autonomous_explorer explorer_node
```
### Save the map
```
ros2 run nav2_map_server map_saver_cli -f my_map
```
You can provide an optional “-f” option to specify the path/name of the map. Make sure you don’t put any extension here, this will be done automatically.

## 🏠 Running in a Custom World

![W2-MadewithClipchamp-ezgif com-optimize](https://github.com/user-attachments/assets/5953d904-3511-4e6e-ba1e-a992fc852d0f)


Place your .world file in the world/ folder.

Update the path in turtlebot3_world.launch.py.

Launch as usual with:
```
ros2 launch autonomous_explorer autonomous_explorer_launch.py

```

## 🤖 Running on a Real TurtleBot3

Start the ROS 2 stack on the robot (bringup + navigation).

Run SLAM Toolbox on your PC or directly on the robot.

Launch the explorer node as shown above (without use_sim_time).

## ✅ Expected Behavior

The TurtleBot3 will autonomously explore unknown areas.

It detects unexplored frontiers (boundaries between known and unknown space).

Uses RRT to plan feasible paths to these frontiers.

Sends goals to Nav2 until the entire map is explored.

Once finished, the robot returns to its start position.

## 📌 Notes

If you want to test with your own robot model, replace turtlebot3 with your robot’s packages.

Works with ROS 2 Humble (tested).

Contributions and improvements are welcome!

