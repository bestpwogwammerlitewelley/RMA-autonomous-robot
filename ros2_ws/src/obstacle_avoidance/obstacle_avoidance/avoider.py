import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Tunable parameters
        self.forward_speed = 0.6      # m/s when path is clear
        self.turn_speed = 0.5         # rad/s when avoiding
        self.stop_distance = 0.4      # m: obstacle threshold ahead
        self.front_arc = 30           # degrees: half-width of the front cone
        self.side_arc = 60            # degrees: width of each side sector

        self.get_logger().info('Obstacle avoider started')

    def sector_min(self, ranges, start_deg, end_deg, num_readings):
        """Minimum valid range within an angular sector (degrees)."""
        deg_per_reading = 360.0 / num_readings
        start_i = int(start_deg / deg_per_reading) % num_readings
        end_i = int(end_deg / deg_per_reading) % num_readings

        if start_i <= end_i:
            sector = ranges[start_i:end_i + 1]
        else:  # wraps past 0
            sector = ranges[start_i:] + ranges[:end_i + 1]

        valid = [r for r in sector if 0.1 < r < 3.5]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        ranges = msg.ranges
        n = len(ranges)

        # Front cone: front_arc degrees either side of straight ahead (index 0)
        front = min(
            self.sector_min(ranges, 0, self.front_arc, n),
            self.sector_min(ranges, 360 - self.front_arc, 360, n),
        )

        # Left and right sectors for choosing turn direction
        left = self.sector_min(ranges, self.front_arc, self.front_arc + self.side_arc, n)
        right = self.sector_min(ranges, 360 - self.front_arc - self.side_arc, 360 - self.front_arc, n)

        if front < self.stop_distance:
            # Obstacle ahead — turn toward whichever side has more space
            cmd.twist.linear.x = 0.0
            if left > right:
                cmd.twist.angular.z = self.turn_speed   # turn left
            else:
                cmd.twist.angular.z = -self.turn_speed  # turn right
        else:
            # Path clear — drive forward
            cmd.twist.linear.x = self.forward_speed
            cmd.twist.angular.z = 0.0

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()