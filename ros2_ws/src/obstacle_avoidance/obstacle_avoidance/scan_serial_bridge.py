#!/usr/bin/env python3
"""
scan_serial_bridge.py — reads TFmini distance from the ESP32 over serial and
publishes it as a single-beam sensor_msgs/LaserScan on /scan.

The ESP32 (cmd_vel_bridge.ino) emits lines:
    D <distance_cm>       (-1 = invalid / out of range)

This node turns each into a LaserScan with ONE ray pointing straight ahead
(angle 0). The avoider consumes /scan unchanged -- with a single beam it can
detect an obstacle ahead but cannot compare left vs right, so it falls back to
turning a fixed direction. That is the expected behaviour for one fixed sensor.

-----------------------------------------------------------------------------
IMPORTANT: shares the serial port with motor_serial_bridge
-----------------------------------------------------------------------------
Both this node and motor_serial_bridge talk to the SAME ESP32 on the SAME
serial device. Two processes cannot open one serial port. Options:
  * Run only ONE of them per port, OR
  * (recommended) merge both into a single node later so one process owns the
    port -- reads "D" lines and writes "V" lines.
For a first bring-up test, run this node to confirm /scan looks right, then
stop it and run motor_serial_bridge separately. See the note at the bottom.

Parameters:
    port      (string, default /dev/ttyUSB0)
    baud      (int,    default 115200)
    frame_id  (string, default laser_frame)
    min_range (float,  default 0.02)   metres
    max_range (float,  default 12.0)   metres  (TFmini-S range)

Run:
    ros2 run obstacle_avoidance scan_serial_bridge
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import serial


class ScanSerialBridge(Node):
    def __init__(self):
        super().__init__('scan_serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 12.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open serial port {port}: {e}\n'
                '  * Is the ESP32 plugged in? (ls /dev/tty*)\n'
                '  * Container started with --device=' + str(port) + ' ?\n'
                '  * Is motor_serial_bridge or the Arduino Serial Monitor '
                'already holding the port? Only one process can open it.')
            raise

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Poll the serial buffer frequently for incoming "D" lines.
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        self.get_logger().info(
            f'scan_serial_bridge started (port={port}, single-beam /scan)')

    def read_serial(self):
        # Drain whatever complete lines are available this tick.
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

        if cm < 0:
            distance = float('inf')          # invalid -> no return
        else:
            distance = cm / 100.0            # cm -> m

        self.publish_scan(distance)

    def publish_scan(self, distance_m):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        # Single ray straight ahead. A tiny non-zero angle span keeps the
        # message well-formed and the avoider's angle sectoring happy.
        scan.angle_min = -0.01
        scan.angle_max = 0.01
        scan.angle_increment = 0.02          # -> exactly 2 points below
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = self.min_range
        scan.range_max = self.max_range

        # Two identical points centred on straight-ahead.
        scan.ranges = [distance_m, distance_m]
        scan.intensities = []

        self.pub.publish(scan)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ScanSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# -----------------------------------------------------------------------------
# SERIAL PORT SHARING
# motor_serial_bridge (writes "V") and this node (reads "D") both need the same
# /dev/ttyUSB0. A serial port can only be opened by one process. For the full
# autonomous run they must be combined into ONE node that owns the port, reads
# "D" lines, and writes "V" commands. Ask for that combined node when ready --
# it is a small merge of the two. For now, test them one at a time.
# -----------------------------------------------------------------------------
