from pros_car_py.env import SERIAL_DEV_DEFAULT
from pros_car_py.car_models import *
import rclpy
from rclpy.node import Node
import orjson
from std_msgs.msg import String
from serial import Serial
from rclpy.duration import Duration

class CarASerialReader(Node):
    def __init__(self):
        super().__init__('car_a_serial_reader')

        serial_port = self.declare_parameter('serial_port', SERIAL_DEV_DEFAULT).value
        self._serial = Serial(serial_port, 115200, timeout=0)

        self.publisher = self.create_publisher(
            String,
            DeviceDataTypeEnum.car_A_state,
            10
        )

        # Create a timer to read from the serial port and publish state every 100 ms
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.log_interval = Duration(seconds=1)  # Log every 1 seconds
        current_time = self.get_clock().now()
        self.last_log_time = current_time

    def timer_callback(self):
        if self._serial.in_waiting:
            incoming_data = self._serial.readline()
            current_time = self.get_clock().now()
            if current_time - self.last_log_time >= self.log_interval:    
                self.get_logger().info(f'Receive from car esp32: {incoming_data}')
                self.last_log_time = current_time

            try:
                # Assuming the incoming data is already in the required JSON format
                
                state_msg = String()
                # validation should be customized
                state_data =dict(CarAState(**orjson.loads(incoming_data)))
                state_msg.data = orjson.dumps(
                    dict(DeviceData(type=DeviceDataTypeEnum.car_A_state,
                        data=state_data))
                    ).decode()
                self.publisher.publish(state_msg)
            except orjson.JSONDecodeError as e:
                self.get_logger().error(f'JSON decode error: {e}')

def main(args=None):
    rclpy.init(args=args)
    car_a_state_publisher = CarASerialReader()
    rclpy.spin(car_a_state_publisher)
    car_a_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
