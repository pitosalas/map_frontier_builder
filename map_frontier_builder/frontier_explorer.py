import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from nav2_msgs.action import NavigateToPose
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import math


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.get_logger().info("Frontier Explorer started")

        # Load configuration from YAML file
        config_path = os.path.join(
            get_package_share_directory('map_frontier_builder'),
            'config',
            'config.yaml'
        )

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Load parameters from config
        self.search_radius = config['frontier_search_radius']
        self.min_size = config['min_frontier_size']
        self.goal_offset = config['goal_offset_distance']

        self.get_logger().info(
            f"Config loaded from {config_path}"
        )
        self.get_logger().info(
            f"  search_radius={self.search_radius}m, "
            f"min_size={self.min_size}, offset={self.goal_offset}m"
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        # Publisher for frontier visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/frontier_markers",
            10
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 action server ready")

        self.current_map = None
        self.navigating = False
        self.current_goal_handle = None

    def map_callback(self, msg):
        self.current_map = msg
        self.get_logger().info(
            f"Map: {msg.info.width}x{msg.info.height}, "
            f"res={msg.info.resolution:.3f}m"
        )

        # Detect frontiers
        frontiers = self.find_frontier_cells(msg)
        self.get_logger().info(f"Found {len(frontiers)} frontier cells")

        # Visualize frontiers
        if len(frontiers) > 0:
            markers = self.create_frontier_markers(frontiers, msg)
            self.marker_pub.publish(markers)
            self.get_logger().info(f"Published {len(markers.markers)} visualization markers")

            # Step 6: Send navigation goal to first frontier (for testing)
            # TODO: Step 5 will add proper frontier selection logic
            if not self.navigating and len(frontiers) > 0:
                self.send_navigation_goal(frontiers[0], msg)

    def find_frontier_cells(self, map_msg):
        """
        Find frontier cells in the occupancy grid.
        A frontier cell is an unknown cell (-1) that is adjacent to a free cell (0).

        Returns:
            List of (x, y) tuples representing frontier cell coordinates in the grid
        """
        width = map_msg.info.width
        height = map_msg.info.height
        data = np.array(map_msg.data).reshape((height, width))

        frontiers = []

        # Iterate through all cells
        for y in range(height):
            for x in range(width):
                # Check if this cell is unknown
                if data[y, x] != -1:
                    continue

                # Check if any adjacent cell is free space
                if self._has_free_neighbor(data, x, y, width, height):
                    frontiers.append((x, y))

        return frontiers

    def _has_free_neighbor(self, data, x, y, width, height):
        """
        Check if cell at (x, y) has at least one free space neighbor.
        Uses 8-connectivity (checks all 8 surrounding cells). A cell is considered a frontier cell if it is unknown (-1) and at least one of its neighbors is free (0). Otherwise it would not be a frontier it would be a far away unknown cell.

        Returns:
            True if at least one neighbor is free (value 0), False otherwise
        """
        # 8-connectivity: check all 8 neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                # Skip the center cell
                if dx == 0 and dy == 0:
                    continue

                nx, ny = x + dx, y + dy

                # Check bounds
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue

                # Check if neighbor is free space
                if data[ny, nx] == 0:
                    return True

        return False

    def create_frontier_markers(self, frontiers, map_msg):
        """
        Create visualization markers for frontier cells.

        Args:
            frontiers: List of (x, y) grid coordinates
            map_msg: OccupancyGrid message for coordinate transformation

        Returns:
            MarkerArray with points representing frontiers
        """
        marker_array = MarkerArray()

        # Create a single marker with all frontier points
        marker = Marker()
        marker.header.frame_id = map_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Size of points
        marker.scale.x = map_msg.info.resolution  # Point width
        marker.scale.y = map_msg.info.resolution  # Point height

        # Color: bright cyan/blue for visibility
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Convert grid coordinates to world coordinates
        for x, y in frontiers:
            point = Point()
            # Transform from grid coordinates to world coordinates
            point.x = map_msg.info.origin.position.x + (x + 0.5) * map_msg.info.resolution
            point.y = map_msg.info.origin.position.y + (y + 0.5) * map_msg.info.resolution
            point.z = 0.0
            marker.points.append(point)

        marker_array.markers.append(marker)
        return marker_array

    def send_navigation_goal(self, frontier_cell, map_msg):
        """
        Send a navigation goal to Nav2 for the given frontier cell.

        Args:
            frontier_cell: (x, y) tuple in grid coordinates
            map_msg: OccupancyGrid message for coordinate transformation
        """
        # Convert grid coordinates to world coordinates
        x, y = frontier_cell
        world_x = map_msg.info.origin.position.x + (x + 0.5) * map_msg.info.resolution
        world_y = map_msg.info.origin.position.y + (y + 0.5) * map_msg.info.resolution

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = map_msg.header.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = world_x
        goal_msg.pose.pose.position.y = world_y
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation (face forward, can be improved later)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending navigation goal to frontier at grid ({x}, {y}) -> "
            f"world ({world_x:.2f}, {world_y:.2f})"
        )

        # Send goal asynchronously
        self.navigating = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected by Nav2")
            self.navigating = False
            return

        self.get_logger().info("Navigation goal accepted by Nav2")
        self.current_goal_handle = goal_handle

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Called periodically with navigation feedback."""
        # Can add progress monitoring here if needed
        pass

    def nav_result_callback(self, future):
        """Called when navigation completes, fails, or is canceled."""
        result = future.result()
        status = result.status

        self.navigating = False
        self.current_goal_handle = None

        # Status codes from action_msgs/GoalStatus
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation goal SUCCEEDED!")
        elif status == 5:  # CANCELED
            self.get_logger().warn("Navigation goal CANCELED")
        elif status == 6:  # ABORTED
            self.get_logger().warn("Navigation goal ABORTED")
        else:
            self.get_logger().warn(f"Navigation goal completed with status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
