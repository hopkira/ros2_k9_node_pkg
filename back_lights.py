import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from back_lights_interfaces.srv import LightsControl, SwitchState  # Custom interfaces
import serial
import ast

class BackLightsNode(Node):
    def __init__(self):
        super().__init__('back_lights_node')

        self.ser = serial.Serial(
            port='/dev/backpanel',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.create_service(Trigger, 'back_lights/on', self.on_handler)
        self.create_service(Trigger, 'back_lights/off', self.off_handler)
        self.create_service(LightsControl, 'back_lights/turn_on', self.turn_on_handler)
        self.create_service(LightsControl, 'back_lights/turn_off', self.turn_off_handler)
        self.create_service(LightsControl, 'back_lights/toggle', self.toggle_handler)
        self.create_service(Trigger, 'back_lights/tv_on', self.tv_on_handler)
        self.create_service(Trigger, 'back_lights/tv_off', self.tv_off_handler)
        self.create_service(SwitchState, 'back_lights/get_switch_state', self.get_switch_state_handler)
        self.create_subscription(String, 'back_lights/cmd', self.cmd_callback, 10)

        self.get_logger().info("BackLightsNode is running.")

    def __write(self, text: str) -> None:
        self.ser.write(str.encode(text + "\n"))

    def __sw_light(self, cmd: str, lights: list) -> None:
        for light in lights:
            text = f"light {light} {cmd}"
            self.__write(text)

    def on_handler(self, request, response):
        self.__write("original")
        response.success = True
        response.message = "Lights turned on"
        return response

    def off_handler(self, request, response):
        self.__write("off")
        response.success = True
        response.message = "Lights turned off"
        return response

    def turn_on_handler(self, request, response):
        self.__sw_light("on", request.lights)
        response.success = True
        response.message = "Lights turned on"
        return response

    def turn_off_handler(self, request, response):
        self.__sw_light("off", request.lights)
        response.success = True
        response.message = "Lights turned off"
        return response

    def toggle_handler(self, request, response):
        self.__sw_light("toggle", request.lights)
        response.success = True
        response.message = "Lights toggled"
        return response

    def tv_on_handler(self, request, response):
        self.__write("tvon")
        response.success = True
        response.message = "TV light on"
        return response

    def tv_off_handler(self, request, response):
        self.__write("tvoff")
        response.success = True
        response.message = "TV light off"
        return response

    def get_switch_state_handler(self, request, response):
        self.__write("switchstate")
        try:
            lines = self.ser.readlines()
            if not lines:
                response.success = False
                response.message = "No response from device"
                response.states = []
                return response

            input_str = lines[0].decode().strip()
            input_str = input_str[len('switchstate:'):]
            switchstate_list = ast.literal_eval(input_str)
            bool_list = [bool(x) for x in switchstate_list]

            response.success = True
            response.message = "Switch states received"
            response.states = bool_list
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            response.states = []

        return response

    def cmd_callback(self, msg):
        self.__write(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = BackLightsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
