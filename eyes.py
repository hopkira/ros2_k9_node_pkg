#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import board
import busio
import adafruit_pca9685


class Eyes:
    def __init__(self, pca):
        self.pca = pca
        self.level = 0.0
        self.set_level(self.level)

    def set_level(self, level: float) -> None:
        level = max(0.0, min(1.0, level))  # Clamp to [0.0, 1.0]
        self.level = level
        value = int(level * 65535)
        self.pca.channels[0].duty_cycle = value

    def off(self) -> None:
        self.set_level(0.0)

    def on(self) -> None:
        self.set_level(1.0)

    def get_level(self) -> float:
        return self.level


class EyeControlNode(Node):
    def __init__(self):
        super().__init__('eye_control_node')

        # Setup PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = adafruit_pca9685.PCA9685(i2c)
        pca.frequency = 60
        self.eyes = Eyes(pca)

        self.create_subscription(Bool, 'is_talking', self.is_talking_callback, 10)
        self.create_subscription(String, 'eye_command', self.eye_command_callback, 10)

        self.talking = False
        self.previous_level = self.eyes.get_level()

        self.get_logger().info("EyeControlNode (without memory) is running.")

    def is_talking_callback(self, msg: Bool):
        if msg.data and not self.talking:
            # Start talking: save current brightness, go full
            self.previous_level = self.eyes.get_level()
            self.eyes.on()
            self.talking = True
            self.get_logger().info("Speaking: Eyes set to full brightness.")
        elif not msg.data and self.talking:
            # Stop talking: revert to saved level
            self.eyes.set_level(self.previous_level)
            self.talking = False
            self.get_logger().info(f"Stopped speaking: Eyes reverted to {self.previous_level:.2f}.")

    def eye_command_callback(self, msg: String):
        if self.talking:
            self.get_logger().info("Ignoring command while speaking.")
            return

        command = msg.data.strip().lower()

        try:
            if command == "on":
                self.eyes.on()
            elif command == "off":
                self.eyes.off()
            elif command.startswith("set_level"):
                # Format: set_level 0.5
                _, val = command.split()
                self.eyes.set_level(float(val))
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to process command '{command}': {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EyeControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
