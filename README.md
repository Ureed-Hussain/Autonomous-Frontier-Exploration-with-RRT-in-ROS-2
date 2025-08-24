# Autonomous-Frontier-Exploration-with-RRT-in-ROS-2
This project implements Autonomous Frontier Exploration on a TurtleBot3 robot in ROS 2 Humble using a Rapidly-exploring Random Tree (RRT) planner.

The robot autonomously explores unknown environments by:

Building a map with SLAM (e.g., slam_toolbox or gmapping).

Detecting unexplored frontiers in the occupancy grid.

Using RRT to plan paths to reachable frontiers.

Sending navigation goals to Nav2 until the environment is fully explored.

Returning to its start position once exploration is complete.

This package works in both simulation (Gazebo) and on a real TurtleBot3.
