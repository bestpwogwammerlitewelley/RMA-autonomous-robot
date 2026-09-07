#!/usr/bin/env python3
"""
robot_serial_bridge.py — single owner of the ESP32 serial link.

Combines the two earlier bridges into one process, because a serial port can
only be opened by ONE process at a time. This node:

  * READS  "D <distance_cm>" lines from the ESP32 -> publishes /scan
  * WRITES "V <linear> <angular>" lines to the ESP32 <- subscribes /cmd_vel

Run THIS for the full autonomous robot, instead of motor_serial_bridge and
scan_serial_bridge separately. Those two remain useful for isolated testing.

-----------------------------------------------------------------------------
/scan is a SINGLE forward-facing beam
-----------------------------------------------------------------------------
The TFmini is one fixed sensor pointing straight ahead, so /scan carries one
ray at angle 0. The avoider can therefore detect an obstacle ahead but cannot
compare left vs right -- it will turn a fixed direction until clear. That is
the expected behaviour for a single fixed rangefinder.

-----------------------------------------------------------------------------
cmd_vel message type
-----------------------------------------------------------------------------
Subscribes to plain geometry_msgs/Twist (hardware default). The avoider must
run with use_stamped_cmd_vel:=false so the types match. Verify with:
    ros2 topic info /cmd_vel --verbose

Parameters:
    port        (string, default /dev/ttyUSB0)
    baud        (int,    default 115200)
    frame_id    (string, default laser_frame)
    min_range   (float,  default 0.02)   metres
    max_range   (float,  default 12.0)   metres
    cmd_rate_hz (float,  default 20.0)   how often the latest /cmd_vel is resent

Run:
    ros2 run obstacle_avoidance robot_serial_bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import serial


class RobotSerialBridge(Node):
    def __init__(self):
        super().__init__('robot_serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('cmd_rate_hz', 20.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        cmd_rate = self.get_parameter('cmd_rate_hz').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open serial port {port}: {e}\n'
                '  * ESP32 plugged in? (ls /dev/tty*)\n'
                '  * Container started with --device=' + str(port) + ' ?\n'
                '  * Anything else holding the port? (Serial Monitor, the old '
                'motor_serial_bridge or scan_serial_bridge) -- only ONE can '
                'own it.')
            raise

        # --- Publisher: /scan from TFmini -----------------------------------
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # --- Subscriber: /cmd_vel -> motors ---------------------------------
        self.linear = 0.0
        self.angular = 0.0
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Poll serial for incoming "D" lines frequently.
        self.read_timer = self.create_timer(0.02, self.read_serial)      # 50 Hz
        # Resend latest velocity so the ESP32 watchdog stays fed.
        self.write_timer = self.create_timer(1.0 / cmd_rate, self.send_command)

        self.get_logger().info(
            f'robot_serial_bridge started (port={port}): /scan in, /cmd_vel out')

    # --- /cmd_vel -> serial -------------------------------------------------
    def cmd_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def send_command(self):
        line = 'V %.3f %.3f\n' % (self.linear, self.angular)
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # --- serial -> /scan ----------------------------------------------------
    def read_serial(self):
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if raw.startswith('D'):
                    self.handle_distance_line(raw)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}')

    def handle_distance_line(self, line):
        parts = line.split()
        if len(parts) != 2:
            return
        try:
            cm = int(parts[1])
        except ValueError:
            return

        distance = float('inf') if cm < 0 else cm / 100.0
        self.publish_scan(distance)

    def publish_scan(self, distance_m):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -0.01
        scan.angle_max = 0.01
        scan.angle_increment = 0.02          # -> exactly 2 points
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = [distance_m, distance_m]
        scan.intensities = []
        self.scan_pub.publish(scan)

    # --- shutdown -----------------------------------------------------------
    def stop_and_close(self):
        try:
            self.ser.write(b'V 0.000 0.000\n')   # explicit stop
            self.ser.flush()
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
