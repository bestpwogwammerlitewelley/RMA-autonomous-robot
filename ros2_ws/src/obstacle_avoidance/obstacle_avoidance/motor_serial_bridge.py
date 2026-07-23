#!/usr/bin/env python3
"""
motor_serial_bridge.py — Pi side of the Pi <-> ESP32 motor link.

Subscribes to /cmd_vel (geometry_msgs/Twist) and forwards each command to the
ESP32 over USB serial as a single line:

    V <linear_m_s> <angular_rad_s>\n

The ESP32 (cmd_vel_bridge.ino) parses that line, converts it to differential
wheel speeds, and drives the TB6612FNG.

-----------------------------------------------------------------------------
WHY IT RESENDS AT A FIXED RATE
-----------------------------------------------------------------------------
The ESP32 runs a 500 ms command watchdog that stops the motors if commands go
quiet. This node therefore resends the latest velocity at rate_hz rather than
only when a /cmd_vel message arrives. Two benefits:

  * Motion stays smooth even if the publisher is slow or bursty.
  * If this node dies, the ROS graph goes down, or the USB cable is pulled,
    commands stop arriving and the robot halts by itself within 500 ms.

-----------------------------------------------------------------------------
PARAMETERS
-----------------------------------------------------------------------------
    port     (string, default /dev/ttyUSB0)  serial device the ESP32 is on
    baud     (int,    default 115200)        must match the ESP32 sketch
    rate_hz  (float,  default 20.0)          resend rate; keep well above the
                                             ESP32's 2 Hz watchdog threshold

Run:
    ros2 run obstacle_avoidance motor_serial_bridge
    ros2 run obstacle_avoidance motor_serial_bridge --ros-args -p port:=/dev/ttyACM0

-----------------------------------------------------------------------------
MESSAGE TYPE
-----------------------------------------------------------------------------
Subscribes to plain geometry_msgs/Twist, matching the avoider's hardware
default (use_stamped_cmd_vel:=false). TwistStamped is sim-only. Verify with:
    ros2 topic info /cmd_vel --verbose
Both the publisher and this subscriber must show Twist, or commands are
silently dropped and the robot will not move.
-----------------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class MotorSerialBridge(Node):
    def __init__(self):
        super().__init__('motor_serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 20.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        rate = self.get_parameter('rate_hz').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open serial port {port}: {e}\n'
                '  * Is the ESP32 plugged in?  (ls /dev/tty* on the Pi host)\n'
                '  * Was the container started with --device=' + str(port) + ' ?\n'
                '  * Is the Arduino Serial Monitor still open? It holds the '
                'port and must be closed.')
            raise

        # Latest commanded velocities, resent at rate_hz.
        self.linear = 0.0
        self.angular = 0.0

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.timer = self.create_timer(1.0 / rate, self.send_command)

        self.get_logger().info(
            f'motor_serial_bridge started (port={port}, baud={baud}, {rate} Hz)')

    def cmd_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def send_command(self):
        line = 'V %.3f %.3f\n' % (self.linear, self.angular)
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def stop_and_close(self):
        """Send an explicit stop so the robot does not coast on shutdown."""
        try:
            self.ser.write(b'V 0.000 0.000\n')
            self.ser.flush()
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialBridge()
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
