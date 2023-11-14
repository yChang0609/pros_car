from pros_car_py.env import SERIAL_DEV_DEFAULT
import rclpy
from rclpy.node import Node
import orjson
from std_msgs.msg import String
from serial import Serial

class CarAStatePublisher(Node):
    def __init__(self):
        super().__init__('car_a_state_publisher')

        serial_port = self.declare_parameter('serial_port', SERIAL_DEV_DEFAULT).value
        self._serial = Serial(serial_port, 115200, timeout=0)

        self.publisher = self.create_publisher(
            String,
            'carA_state',
            10
        )

        # Create a timer to read from the serial port and publish state every 100 ms
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self._serial.in_waiting:
            incoming_data = self._serial.readline()
            try:
                # Assuming the incoming data is already in the required JSON format
                state_data = orjson.loads(incoming_data.decode().strip())
                state_msg = String()
                state_msg.data = orjson.dumps(state_data).decode()
                self.publisher.publish(state_msg)
            except orjson.JSONDecodeError as e:
                self.get_logger().error(f'JSON decode error: {e}')

def main(args=None):
    rclpy.init(args=args)
    car_a_state_publisher = CarAStatePublisher()
    rclpy.spin(car_a_state_publisher)
    car_a_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
