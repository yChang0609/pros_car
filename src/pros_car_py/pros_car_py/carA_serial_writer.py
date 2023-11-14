from pros_car_py.env import SERIAL_DEV_DEFAULT
from pros_car_py.car_models import CARA_CONTROL, TwoWheelAndServoControlSignal
import rclpy
from rclpy.node import Node
import orjson
from std_msgs.msg import String
from serial import Serial

class CarAControlSubscriber(Node):
    def __init__(self):
        super().__init__('car_a_control_subscriber')
        self.subscription = self.create_subscription(
            String,
            'carA_control', # topic name
            self.listener_callback,
            10
        )
        serial_port = self.declare_parameter('serial_port', SERIAL_DEV_DEFAULT).value

        self.subscription  # prevent unused variable warning
        self._serial = Serial(serial_port, 115200, timeout=0)


    def listener_callback(self, msg):
        try:
            control_data = orjson.loads(msg.data)
            
            if control_data.get('type') == CARA_CONTROL:
                self.process_control_data(control_data.get('data', {}))
        except orjson.JSONDecodeError as e:
            self.get_logger().error('JSON decode error: {}'.format(e))
        except KeyError as e:
            self.get_logger().error('Missing key in JSON data: {}'.format(e))

    def process_control_data(self, data):
        # Process the control data as needed
        # direction = data.get('direction')
        # target_vel = data.get('target_vel')
        # TODO should be customized
        control_signal = TwoWheelAndServoControlSignal(**orjson.loads(data))

        self._serial.write(orjson.dumps(control_signal.json(),option=orjson.OPT_APPEND_NEWLINE))
        # self.get_logger().info(f'Received type:{type(data)} data: {data}')
        self.get_logger().info(f'Received {control_signal}')


def main(args=None):
    rclpy.init(args=args)
    car_a_control_subscriber = CarAControlSubscriber()
    rclpy.spin(car_a_control_subscriber)
    car_a_control_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
