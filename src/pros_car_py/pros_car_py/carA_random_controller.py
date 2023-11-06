
import json

from pydantic import PydanticValueError
from geometry_msgs.msg import Twist
from serial import Serial
from .env import SERIAL_DEV_DEFAULT
from rclpy.node import Node
import rclpy
from .car_models import CarAControlSignal, CarAState
from rclpy.duration import Duration
import random


class CarARandomController(Node):
    def __init__(self):
        super().__init__('carA_random_controller')

        # Set up the serial connection
        serial_port = self.declare_parameter('serial_port', SERIAL_DEV_DEFAULT).value
        self._serial = Serial(serial_port, 115200, timeout=0)

        # Create a timer to read data from ESP32 every 0.01 milliseconds (0.01 seconds)
        self.read_timer = self.create_timer(0.01, self.read_from_esp32_callback)

        # Create a timer to send data to ESP32 every 2000 milliseconds (2 second)
        self.write_timer = self.create_timer(5.0, self.send_to_esp32_callback)
        self.last_log_time = self.get_clock().now()
        self.log_interval = Duration(seconds=1)  # Log every 1 seconds


    def read_from_esp32_callback(self):
        # This function is called to read data from the ESP32
        if self._serial.in_waiting:
            incoming_data = self._serial.readline()
            # Deserialize the JSON string to CarAState object
            # log data in a duration
            current_time = self.get_clock().now()
            if current_time - self.last_log_time >= self.log_interval:
                try:
                    state_data = json.loads(incoming_data.decode().strip())
                    car_state = CarAState(**state_data)
                    self.get_logger().info(f"Received state: {car_state.json()}")
                except (json.JSONDecodeError, PydanticValueError) as e:
                    self.get_logger().error(f"Failed to decode state data: {e}")
                self.last_log_time = current_time

    def send_to_esp32_callback(self):
        # This function is called every 1 second to send data to the ESP32
        command = self._generate_random_command()
        command_str = command.json()
        self._serial.write(command_str.encode('utf-8'))
        # Log the sent command for debugging purposes
        self.get_logger().info(f"Sent command: {command_str}")

    def _generate_random_command(self):
        # This function generates a random command to send to the ESP32
        random_velocity = [random.uniform(-10, 10) for _ in range(2)]
        random_direction = random.randint(70, 110)
        control_signal = CarAControlSignal(target_vel=random_velocity, direction=random_direction)
        return control_signal

def main(args=None):
    rclpy.init(args=args)
    serial_writer = CarARandomController()
    rclpy.spin(serial_writer)

    serial_writer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
