import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorGps, VehicleLocalPosition, TrajectorySetpoint
from shapely.geometry import Point, Polygon

class BoundaryChecker(Node):
    def __init__(self):
        super().__init__('boundary_checker')
        print("Function Called")
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

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to the GPS topic
        self.vehicle_gps_position = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile
        )

    def gps_callback(self, msg):
        print("GPS CALLED")
        """Callback for GPS data."""
        current_position = Point(msg.longitude_deg, msg.latitude_deg)
        print("Longitude:" + str(msg.longitude_deg))
        print("Latitude:" + str(msg.latitude_deg))

        current_position = Point(-112.7385000,50.1010000)

        if self.is_within_boundary(current_position):
            self.get_logger().info('Drone is within the boundary.')
        else:
            self.get_logger().warning('Drone is outside the boundary!')

    def is_within_boundary(self, point):
        """Check if the given point is inside the boundary."""
        return self.boundary_polygon.contains(point)

def main(args=None):
    print("Running")
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
