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
        self.get_logger().info('Obstacle avoider started')

    def scan_callback(self, msg):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        ranges = msg.ranges
        num_readings = len(ranges)
        front_readings = ranges[0:15] + ranges[num_readings-15:]
        valid = [r for r in front_readings if 0.1 < r < 3.5]

        if not valid:
            cmd.twist.linear.x = 0.45
        else:
            min_distance = min(valid)
            if min_distance < 0.4:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.5
            else:
                cmd.twist.linear.x = 0.45
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