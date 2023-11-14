
from rclpy.node import Node
import rclpy
from pros_car_py.car_models import *
from rclpy.duration import Duration
import orjson
from std_msgs.msg import String
import random

class CarARandomAI(Node):
    def __init__(self):
        super().__init__('car_a_random_ai')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            DeviceDataTypeEnum.car_A_state,
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            DeviceDataTypeEnum.car_A_control,# topic name
            10
        )
        self.pub_timer = self.create_timer(2, self._pub_callback)

        self.log_interval = Duration(seconds=1)  # Log every 1 second
        self.last_log_time = self.get_clock().now()

    def _sub_callback(self, msg):
        # Process the incoming message (if needed)
        current_time = self.get_clock().now()
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(f'receive msg {msg}')
            self.last_log_time = current_time

    def _pub_callback(self):
        # Generate a random control signal
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_A_control),
            "data":dict(CarAControl(
                    direction=random.randint(70, 110),
                    target_vel= [random.uniform(-20, 20), random.uniform(-20,20)]
                ))
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)

        self.get_logger().info(f'publish {control_msg}')
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
