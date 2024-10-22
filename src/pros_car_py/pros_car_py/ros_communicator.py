from rclpy.node import Node
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseStamped,
    Twist,
    TransformStamped,
    Point,
)
from std_msgs.msg import Header, Float32, String
from sensor_msgs.msg import LaserScan
class RosCommunicator(Node):
    def __init__(self):
        super().__init__("RosCommunicator")

        self.latest_amcl_pose = None
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscriber_amcl_callback, 10
        )
        self.subscriber_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.subscriber_goal_callback, 1
        )
        
        self.latest_lidar = None
        self.subscriber_lidar = self.create_subscription(
            LaserScan, "/scan", self.subscriber_lidar_callback, 1
        )

        self.subscriber_object_direction = self.create_subscription(
            String,
            "/object_direction",
            self.subscriber_object_direction_callback,
            10,
        )

        self.publisher_rear = self.create_publisher(String, DeviceDataTypeEnum.car_C_rear_wheel, 10)
        self.publisher_forward = self.create_publisher(String, DeviceDataTypeEnum.car_C_front_wheel, 10)

    def subscriber_amcl_callback(self, msg):
        self.latest_amcl_pose = msg

    def get_latest_amcl_pose(self):
        if self.latest_amcl_pose is None:
            self.get_logger().warn("No AMCL pose data received yet.")
        return self.latest_amcl_pose
    
    def subscriber_lidar_callback(self, msg):
        self.latest_lidar = msg

    def get_latest_lidar(self):
        if self.latest_lidar is None:
            self.get_logger().warn("No Lidar data received yet.")
        return self.latest_lidar
    
