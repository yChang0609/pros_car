import orjson
from rclpy.node import Node
from std_msgs.msg import String
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl


class CarController(Node):
    """控制車輛的類別"""

    def __init__(self, vel: float = 10):
        super().__init__("car_controller")
        self.vel = vel
        self._vel1 = 0  # rad/s
        self._vel2 = 0
        self._vel3 = 0
        self._vel4 = 0

        # ROS Publisher
        self.publisher = self.create_publisher(
            String, DeviceDataTypeEnum.car_C_rear_wheel, 10
        )
        self.publisher_forward = self.create_publisher(
            String, DeviceDataTypeEnum.car_C_front_wheel, 10
        )

    def update_velocity(self, vel1, vel2, vel3, vel4):
        """更新車子的速度"""
        self._vel1 = vel1
        self._vel2 = vel2
        self._vel3 = vel3
        self._vel4 = vel4

    def publish_control(self):
        """發佈車輛控制訊號"""
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_C_rear_wheel),
            "data": dict(CarCControl(target_vel=[self._vel1, self._vel2])),
        }
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()
        self.publisher.publish(control_msg)

        control_signal_forward = {
            "type": str(DeviceDataTypeEnum.car_C_front_wheel),
            "data": dict(CarCControl(target_vel=[self._vel3, self._vel4])),
        }
        control_msg_forward = String()
        control_msg_forward.data = orjson.dumps(control_signal_forward).decode()
        self.publisher_forward.publish(control_msg_forward)

        self.get_logger().info(f"Published: {control_msg}")
        self.get_logger().info(f"Published to forward: {control_msg_forward}")
