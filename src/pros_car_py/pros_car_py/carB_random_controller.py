
import json

from pydantic import PydanticValueError
from serial import Serial
from .env import SERIAL_FRONT_DEFAULT,SERIAL_BACK_DEFAULT
from rclpy.node import Node
import rclpy
from .car_models import TwoWheelControlSignal, TwoWheelState
from rclpy.duration import Duration
import random


class CarBRandomController(Node):
    def __init__(self):
        super().__init__('carA_random_controller')

        # Set up the serial connection
        front_serial_port = self.declare_parameter('front_serial_port', SERIAL_FRONT_DEFAULT).value
        back_serial_port = self.declare_parameter('back_serial_port', SERIAL_BACK_DEFAULT).value
        self._front_serial = Serial(front_serial_port, 115200, timeout=0)
        self._back_serial = Serial(back_serial_port, 115200, timeout=0)

        # Create a timer to read data from ESP32 every 0.01 milliseconds (0.01 seconds)
        self.read_timer = self.create_timer(0.01, self.read_from_esp32_callback)

        # Create a timer to send data to ESP32 every 2000 milliseconds (2 second)
        self.write_timer = self.create_timer(5.0, self.send_to_esp32_callback)
        self.last_log_time = self.get_clock().now()
        self.log_interval = Duration(seconds=1)  # Log every 1 seconds


    def read_from_esp32_callback(self):
        # This function is called to read data from the ESP32
        if self._front_serial.in_waiting:
            back_serial_data = self._back_serial.readline()
            front_serial_data = self._front_serial.readline()
            # Deserialize the JSON string to CarBState object
            # log data in a duration
            current_time = self.get_clock().now()
            if current_time - self.last_log_time >= self.log_interval:
                try:
                    front_state_data = json.loads(front_serial_data.decode().strip())
                    back_state_data = json.loads(back_serial_data.decode().strip())
                    front_state = TwoWheelState(**front_state_data)
                    back_state = TwoWheelState(**back_state_data)
                    self.get_logger().info(f"Received front state: {front_state.json()}")
                    self.get_logger().info(f"Received back state: {back_state.json()}")
                except (json.JSONDecodeError, PydanticValueError) as e:
                    self.get_logger().error(f"Failed to decode state data: {e}")
                self.last_log_time = current_time

    def send_to_esp32_callback(self):
        # This function is called every 1 second to send data to the ESP32
        command_str = self._generate_random_command_str()
        self._front_serial.write(command_str.encode('utf-8'))
        self.get_logger().info(f"Sent command to front wheel: {command_str}")
        # Log the sent command for debugging purposes
        command_str = self._generate_random_command_str()
        self._back_serial.write(command_str.encode('utf-8'))
        self.get_logger().info(f"Sent command to back wheel: {command_str}")

    def _generate_random_command_str(self):
        # This function generates a random command to send to the ESP32
        random_velocity = [random.uniform(-10, 10) for _ in range(2)]
        control_signal = TwoWheelControlSignal(target_vel=random_velocity)
        return control_signal.json()

def main(args=None):
    rclpy.init(args=args)
    serial_writer = CarBRandomController()
    rclpy.spin(serial_writer)

    serial_writer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
