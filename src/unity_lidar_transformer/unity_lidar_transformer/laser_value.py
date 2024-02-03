from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile
import math
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import time
import rclpy

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar_value')
        self.get_logger().info("on_configure() is called.")
        self.subscription_ = self.create_subscription(Float32MultiArray, "lidar_value", self.callback, 10)
        # 添加LaserScan publisher
        qos_profile = QoSProfile(depth=10)
        self.laserscan_publisher = self.create_publisher(LaserScan, '/scan', qos_profile)        
    
    def callback(self, msg):
        # self.get_logger().info("Get message")
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / len(msg.data)
        scan.time_increment = 0.0
        scan.range_min = 0.0
        scan.range_max = 100.0
        scan.ranges = msg.data
        self.laserscan_publisher.publish(scan)
        # time.sleep(0.05)
        
def main(args=None):
    rclpy.init(args=args)
    node = Lidar()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
