#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_data = None
        self.current_pose = None

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Received warehouse map')

    def lidar_callback(self, msg):
        # Basic obstacle detection
        obstacles = [r for r in msg.ranges if r < 0.5]  # Detect within 0.5m
        if obstacles:
            self.get_logger().warn('Obstacle detected, replanning path')
            self.plan_path()

    def plan_path(self):
        if not self.map_data or not self.current_pose:
            return
        # Placeholder A* path planning
        path = Path()
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 10.0  # Example goal
        goal_pose.pose.position.y = 10.0
        path.poses.append(goal_pose)
        self.path_pub.publish(path)
        self.get_logger().info('Published new path')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
