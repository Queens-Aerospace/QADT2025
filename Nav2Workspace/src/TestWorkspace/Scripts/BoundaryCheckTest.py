import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point, Polygon

class BoundaryChecker(Node):
    def __init__(self):
        super().__init__('boundary_checker')

        self.boundaryCoords = [
            (-110.7328077, 50.0970884),
            (-110.7327756, 50.1061216),
            (-110.7437887, 50.1061482),
            (-110.7437798, 50.1035232),
            (-110.7382540, 50.0988785),
            (-110.7382533, 50.0971194),
            (-110.7328077, 50.0970884) 
        ]

        
        self.boundary_polygon = Polygon(self.boundaryCoords)

        # Subscribe to the GPS topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/rmu/out/vehicle_gps_position',  # Adjust the topic name as per your setup
            self.gps_callback,
            10
        )

    def gps_callback(self, msg):
        """Callback for GPS data."""
        current_position = Point(msg.longitude, msg.latitude)

        if self.is_within_boundary(current_position):
            self.get_logger().info('Drone is within the boundary.')
        else:
            self.get_logger().warning('Drone is outside the boundary!')

    def is_within_boundary(self, point):
        """Check if the given point is inside the boundary."""
        return self.boundary_polygon.contains(point)

def main(args=None):
    rclpy.init(args=args)

    boundary_checker = BoundaryChecker()

    try:
        rclpy.spin(boundary_checker)
    except KeyboardInterrupt:
        pass
    boundary_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
