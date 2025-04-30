import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import serial
import json
import math
import time

class EarsNode(Node):
    def __init__(self):
        super().__init__('ears_node')

        self.ser = serial.Serial(
            port='/dev/ears',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=10
        )
        self.following = False

        # Subscribe to ear commands
        self.command_sub = self.create_subscription(
            String,
            'ears_command',
            self.command_callback,
            10
        )

        # Publishers
        self.follow_dist_pub = self.create_publisher(Float32, 'ears/follow_distance', 10)
        self.rotate_result_pub = self.create_publisher(Bool, 'ears/rotate_safe', 10)

        self.get_logger().info('Ears node started.')

    def command_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'stop':
            self.stop()
        elif cmd == 'scan':
            self.scan()
        elif cmd == 'fast':
            self.fast()
        elif cmd == 'think':
            self.think()
        elif cmd == 'follow_read':
            dist = self.follow_read()
            self.follow_dist_pub.publish(Float32(data=dist))
        elif cmd == 'safe_rotate':
            safe = self.safe_rotate()
            self.rotate_result_pub.publish(Bool(data=safe))
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    def __write(self, text: str):
        self.get_logger().info(f"Ears command: {text}()")
        self.ser.write(str.encode(text + "()\n"))

    def stop(self):
        self.following = False
        self.__write("stop")

    def scan(self):
        self.following = False
        self.__write("scan")

    def fast(self):
        self.following = False
        self.__write("fast")

    def think(self):
        self.following = False
        self.__write("think")

    def follow_read(self) -> float:
        if not self.following:
            self.__write("follow")
            self.following = True
        json_reading = self.ser.readline().decode("ascii")
        reading = json.loads(json_reading)
        return reading['distance']

    def safe_rotate(self) -> bool:
        safe_x = 0.3
        safe_y = 0.6
        duration = 4
        detected = False
        self.__write("fast")
        end_time = time.time() + duration

        while time.time() < end_time and not detected:
            try:
                json_reading = self.ser.readline().decode("ascii")
                reading = json.loads(json_reading)
                dist = reading['distance']
                angle = reading['angle']
                x = abs(dist * math.cos(angle))
                y = abs(dist * math.sin(angle))
                if x <= safe_x and y <= safe_y:
                    detected = True
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

        self.__write("stop")
        return not detected

def main(args=None):
    rclpy.init(args=args)
    node = EarsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
