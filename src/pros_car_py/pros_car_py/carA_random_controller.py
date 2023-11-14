
import json

from pydantic import PydanticValueError
from geometry_msgs.msg import Twist
from serial import Serial
from .env import SERIAL_DEV_DEFAULT
from rclpy.node import Node
import rclpy
from .car_models import TwoWheelAndServoControlSignal, TwoWheelAndServoState
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
        self.log_interval = Duration(seconds=1)  # Log every 1 seconds

import rclpy
from rclpy.node import Node
import orjson
from std_msgs.msg import String
import random

class CarARandomAI(Node):
    def __init__(self):
        super().__init__('car_a_random_ai')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'carA_state',
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            'carA_control',
            10
        )
        self.pub_timer = self.create_timer(2, self._pub_callback)

        self.log_interval = Duration(seconds=1)  # Log every 1 seconds

    def _sub_callback(self, msg):
        # Process the incoming message (if needed)
        self.get_logger().info(f"Received state: {msg.data}")

    def _pub_callback(self):
        # Generate a random control signal
        control_signal = {
            "type": "carA_control",
            "data":TwoWheelAndServoControlSignal(
                    direction=random.randint(70, 110),
                    target_vel= [random.uniform(-20, 20), random.uniform(-20,20)]
                )
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)
        self.get_logger().info(f"Published control signal: {control_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    car_a_random_ai = CarARandomAI()
    rclpy.spin(car_a_random_ai)
    car_a_random_ai.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
