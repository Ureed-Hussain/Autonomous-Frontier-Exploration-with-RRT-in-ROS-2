import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from .frontier_detector import detect_frontiers  # You already implemented this
import numpy as np
import math
from .rrt_planner import rrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.rrt_path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.start_pose = None


        # Navigation action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Internal state
        self._navigating = False
        self.map = None
        self.map_info = None
        self.robot_pose = (0.0, 0.0)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_pose is None:
            self.start_pose = (x, y)
            self.get_logger().info(f"Start position saved at ({x:.2f}, {y:.2f})")
        self.robot_pose = (x, y)
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.get_logger().debug(f"Robot position: {self.robot_pose}")

    def map_callback(self, msg):
        if self._navigating:
            return  # Wait for current goal to complete

        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        frontiers = detect_frontiers(msg)
        if not frontiers:
            self.get_logger().info("No frontiers found.")
            return

        # Filter out bad frontiers (e.g., near wall or unknown space)
        good_frontiers = self.filter_frontiers(frontiers)

        if not good_frontiers:
            self.get_logger().info("No valid frontiers after filtering.")
            self.get_logger().info("Exploration complete. Returning to start...")
            self.return_to_start()

            return

        # Select best frontier (nearest to robot)
        # goal_x, goal_y = self.select_best_frontier(good_frontiers)
        goal = self.select_best_frontier(good_frontiers)
        if not goal:
            self.get_logger().info("RRT failed to find path to any frontier.")
            return
        goal_x, goal_y = goal


        self.get_logger().info(f"Navigating to frontier: x={goal_x:.2f}, y={goal_y:.2f}")
        self.send_goal(goal_x, goal_y)

    def filter_frontiers(self, frontiers, threshold=3):
        """Remove frontiers that are too close to walls or robot"""
        valid = []
        for fx, fy in frontiers:
            # Check distance from robot
            dist = math.hypot(fx - self.robot_pose[0], fy - self.robot_pose[1])
            
            if dist < 0.70:
                continue  # Skip if too close

            # Convert to map index
            mx = int((fx - self.map_info.origin.position.x) / self.map_info.resolution)
            my = int((fy - self.map_info.origin.position.y) / self.map_info.resolution)

            if self.is_near_wall(mx, my, threshold):
                continue

            valid.append((fx, fy))
        return valid



    def is_near_wall(self, x, y, threshold):
        for dx in range(-threshold, threshold + 1):
            for dy in range(-threshold, threshold + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map.shape[1] and 0 <= ny < self.map.shape[0]:
                    if self.map[ny][nx] == 100:
                        return True
        return False

    # def select_best_frontier(self, frontiers):
    #     """Select nearest frontier to robot pose"""
    #     rx, ry = self.robot_pose
    #     best = min(frontiers, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
    #     return best
    def select_best_frontier(self, frontiers):
        """Use RRT to plan path to reachable frontier"""
        for fx, fy in sorted(frontiers, key=lambda p: math.hypot(p[0] - self.robot_pose[0], p[1] - self.robot_pose[1])):
            path = rrt(self.robot_pose, (fx, fy), self.map, self.map_info.resolution, self.map_info.origin)
            self.publish_rrt_path(path)  # path is the list of (x, y) points returned from RRT
            if path:
                return path[-1]  # Return last point in the path
        return None


    def send_goal(self, x, y):
        self.current_goal = (x, y)  # Store goal to compare later
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward

        self._action_client.wait_for_server()
        self._navigating = True
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            self._navigating = False
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Goal reached.")
        self._navigating = False
        if self.current_goal and self.is_returning_home(*self.current_goal):
            self.get_logger().info("Exploration complete. Everything complete — please save the map now.")

    def publish_rrt_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # Facing forward
            path_msg.poses.append(pose)

        self.rrt_path_pub.publish(path_msg)
    def return_to_start(self):
        if self.start_pose:
            x, y = self.start_pose
            self.send_goal(x, y)
        else:
            self.get_logger().warn("Start pose not set. Cannot return.")

    def is_returning_home(self, x, y, threshold=0.3):
        if self.start_pose is None:
            return False
        sx, sy = self.start_pose
        return math.hypot(sx - x, sy - y) < threshold


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
