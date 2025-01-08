#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
# import numpy as np
# import os
# import subprocess

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        #Automatically set permissions
        #self.set_serial_permissions('/dev/ttyUSB0')

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        #self.serial_port = serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 115200, timeout=1)

    # def set_serial_permissions(self, device_path):
    #         try:
    #             # Change permissions using os.chmod (only if necessary)
    #             if not os.access(device_path, os.W_OK):
    #                 self.get_logger().info(f"Setting permissions for {device_path}")
    #                 subprocess.run(['chmod', '666', device_path], check=True)
    #         except Exception as e:
    #             self.get_logger().error(f"Failed to set permissions for {device_path}: {e}")

    def cmd_vel_callback(self, msg):
        # Convert cmd_vel to Arduino command
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_dist = 0.075  # meters
        wheel_radius = 0.0325  # meters

        # vL = (linear - angular * wheel_base / 2)/wheel_radius
        # vR = (linear + angular * wheel_base / 2)/wheel_radius

        # vL_rpm = vL*60/(2*np.pi)
        # vR_rpm = vR*60/(2*np.pi)

        # Lpwm = 3.46*vL_rpm - 4.83
        # Rpwm = 3.46*vR_rpm - 4.83

        self.vL = (2 * linear - angular * wheel_dist) / 2 * wheel_radius
        self.vR = (2 * linear + angular * wheel_dist) / 2 * wheel_radius

        # Send command to Arduino
        command = f"{vL},{vR}\n"
        self.get_logger().info(f"Sending to Arduino: {command}")
        self.serial_port.write(command.encode())

    def publish_joint_states(self):
        # Read from Arduino
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Received from Arduino: {data}")
            positions = data.split(',')

            if len(positions) == 2:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
                # joint_state.velocity = [float(positions[0]), float(positions[1])]
                joint_state.velocity = [self.vL, self.vR]
                joint_state.position = [float(positions[0]), float(positions[1])]
                self.publisher.publish(joint_state)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArduinoBridge()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.publish_joint_states()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
