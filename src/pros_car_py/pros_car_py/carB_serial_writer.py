from pros_car_py.env import SERIAL_DEV_DEFAULT, SERIAL_DEV_FORWARD_DEFAULT
from pros_car_py.car_models import *
import rclpy
from rclpy.node import Node
import orjson
from std_msgs.msg import String
from serial import Serial


class CarBControlSubscriber(Node):
    def __init__(self):
        super().__init__('car_b_control_subscriber')
        self.subscription = self.create_subscription(
            String,
            DeviceDataTypeEnum.car_B_control,  # topic name
            self.listener_callback,
            10
        )
        serial_port = self.declare_parameter('serial_port', SERIAL_DEV_DEFAULT).value

        self.subscription  # prevent unused variable warning
        self._serial = Serial(serial_port, 115200, timeout=0)
        # ------------------------------------------------------------------

    def listener_callback(self, msg):
        try:
            control_data = orjson.loads(msg.data)
            # TODO use more clear method to write
            # TODO divide serial data and data validation
            if control_data.get('type') == DeviceDataTypeEnum.car_B_control:
                self.process_control_data(CarDControl, control_data.get('data', {}))
        except orjson.JSONDecodeError as e:
            self.get_logger().error('JSON decode error: {}'.format(e))
        except KeyError as e:
            self.get_logger().error('Missing key in JSON data: {}'.format(e))

    def process_control_data(self, type_cls, data: dict):

        # TODO should be customized
        control_signal = type_cls(**data)

        self._serial.write(orjson.dumps(dict(control_signal), option=orjson.OPT_APPEND_NEWLINE))
        self.get_logger().info(f'Received {control_signal}')


def main(args=None):
    rclpy.init(args=args)
    car_b_control_subscriber = CarBControlSubscriber()
    rclpy.spin(car_b_control_subscriber)
    car_b_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
