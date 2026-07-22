#!/usr/bin/env python3
"""
avoider.py — obstacle-avoidance node.

Subscribes to /scan (sensor_msgs/LaserScan) and publishes velocity commands to
/cmd_vel. Depends ONLY on the scan message and /cmd_vel, with no sensor-specific
code, so the SAME node runs unchanged on:
  * the Gazebo lidar in simulation (hundreds of readings), and
  * the real robot's five HC-SR04 ultrasonics, fused into a small LaserScan by
    the serial bridge.

The node adapts to scan resolution (it sectors readings by ANGLE, so 5 beams or
500 both work) and to the /cmd_vel message type expected by its consumer.

-----------------------------------------------------------------------------
cmd_vel message type — the `use_stamped_cmd_vel` parameter
-----------------------------------------------------------------------------
Whether /cmd_vel carries geometry_msgs/Twist or geometry_msgs/TwistStamped is a
property of whatever CONSUMES /cmd_vel, not of the ROS distro:

  * SIM   (Jazzy):  Gazebo/TurtleBot3 bridge expects TwistStamped -> use_stamped_cmd_vel:=true
  * ROBOT (Humble): our motor bridge expects plain Twist          -> use_stamped_cmd_vel:=false  (default)

Team rule: sim = TwistStamped, hardware = Twist, and this node takes the flag so
neither side has to be rewritten.

Run examples:
  ros2 run obstacle_avoidance avoider                                          # default: Twist (hardware)
  ros2 run obstacle_avoidance avoider --ros-args -p use_stamped_cmd_vel:=true  # sim
-----------------------------------------------------------------------------
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # --- cmd_vel message type selection --------------------------------
        # Default false (plain Twist) = hardware/Humble. Set true for sim/Jazzy.
        self.declare_parameter('use_stamped_cmd_vel', False)
        self.use_stamped = self.get_parameter('use_stamped_cmd_vel').value

        msg_type = TwistStamped if self.use_stamped else Twist
        self.publisher = self.create_publisher(msg_type, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # --- Tunable parameters --------------------------------------------
        self.forward_speed = 0.2     # m/s when path is clear
        self.turn_speed = 0.5        # rad/s when avoiding
        self.stop_distance = 0.4     # m: obstacle threshold ahead
        self.front_arc = 30.0        # deg: half-width of the front cone
        self.side_arc = 60.0         # deg: width of each side sector beyond the front

        self.get_logger().info(
            'Obstacle avoider started (cmd_vel type: %s)'
            % ('TwistStamped' if self.use_stamped else 'Twist'))

    def sector_min(self, msg, low_deg, high_deg):
        """
        Minimum valid range among readings whose angle falls in [low_deg, high_deg].
        Works for any scan resolution by mapping each index to its real angle via
        angle_min / angle_increment, rather than assuming a fixed array size.
        Returns inf if no valid reading falls in the sector.
        """
        low = math.radians(low_deg)
        high = math.radians(high_deg)
        best = float('inf')

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # Normalise to [-pi, pi] for consistent comparison.
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if low <= angle <= high:
                if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                    if r < best:
                        best = r
        return best

    def make_cmd(self, linear_x, angular_z):
        """Build the correct message type with the given velocities."""
        if self.use_stamped:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.twist.linear.x = linear_x
            cmd.twist.angular.z = angular_z
            return cmd
        else:
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            return cmd

    def scan_callback(self, msg):
        # Front cone: front_arc degrees either side of straight ahead.
        front = self.sector_min(msg, -self.front_arc, self.front_arc)

        # Side sectors for choosing turn direction.
        left = self.sector_min(msg, -self.front_arc - self.side_arc, -self.front_arc)
        right = self.sector_min(msg, self.front_arc, self.front_arc + self.side_arc)

        if front < self.stop_distance:
            # Obstacle ahead — turn toward whichever side has more clearance.
            if left > right:
                cmd = self.make_cmd(0.0, self.turn_speed)    # more room left -> turn left
            else:
                cmd = self.make_cmd(0.0, -self.turn_speed)   # more room right -> turn right
        else:
            # Path clear — drive forward.
            cmd = self.make_cmd(self.forward_speed, 0.0)

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
