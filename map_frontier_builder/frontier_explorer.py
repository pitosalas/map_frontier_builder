import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.get_logger().info("Frontier Explorer started")

        self.declare_parameter("frontier_search_radius", 5.0)
        self.declare_parameter("min_frontier_size", 10)
        self.declare_parameter("goal_offset_distance", 0.5)

        self.search_radius = self.get_parameter("frontier_search_radius").value
        self.min_size = self.get_parameter("min_frontier_size").value
        self.goal_offset = self.get_parameter("goal_offset_distance").value

        self.get_logger().info(
            f"Config: search_radius={self.search_radius}m, "
            f"min_size={self.min_size}, offset={self.goal_offset}m"
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        self.current_map = None

    def map_callback(self, msg):
        self.current_map = msg
        self.get_logger().info(
            f"Map: {msg.info.width}x{msg.info.height}, "
            f"res={msg.info.resolution:.3f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
