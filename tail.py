#!/usr/bin/env python3

import time
import board
import busio
import adafruit_pca9685

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Tail:
    def __init__(self, pca):
        self.pca = pca
        self.center()

    def wag_h(self):
        self.pca.channels[4].duty_cycle = 5121
        for _ in range(4):
            self.pca.channels[5].duty_cycle = 5201  # left
            time.sleep(0.25)
            self.pca.channels[5].duty_cycle = 7042  # right
            time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 5601  # center

    def wag_v(self):
        self.pca.channels[5].duty_cycle = 5601
        for _ in range(4):
            self.pca.channels[4].duty_cycle = 5921  # down
            time.sleep(0.25)
            self.pca.channels[4].duty_cycle = 4321  # up
            time.sleep(0.25)
        self.pca.channels[4].duty_cycle = 5121

    def center(self):
        self.pca.channels[4].duty_cycle = 5121
        self.pca.channels[5].duty_cycle = 5601

    def up(self):
        self.pca.channels[5].duty_cycle = 5601
        self.pca.channels[4].duty_cycle = 4321

    def down(self):
        self.pca.channels[5].duty_cycle = 5601
        self.pca.channels[4].duty_cycle = 5921

class TailNode(Node):
    def __init__(self):
        super().__init__('tail_node')

        # Setup PCA9685 and Tail class
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = adafruit_pca9685.PCA9685(i2c)
        pca.frequency = 60
        self.tail = Tail(pca)

        # ROS subscriber
        self.subscription = self.create_subscription(
            String,
            'tail_command',
            self.command_callback,
            10
        )

        self.get_logger().info("TailNode is ready and listening for commands.")

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {command}")

        actions = {
            'wag_h': self.tail.wag_h,
            'wag_v': self.tail.wag_v,
            'center': self.tail.center,
            'up': self.tail.up,
            'down': self.tail.down,
        }

        action = actions.get(command)
        if action:
            action()
        else:
            self.get_logger().warn(f"Unknown tail command: '{command}'")

def main(args=None):
    rclpy.init(args=args)
    node = TailNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
