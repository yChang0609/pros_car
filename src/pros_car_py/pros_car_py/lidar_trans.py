import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarRePublisher(Node):

    def __init__(self):
        super().__init__('lidar_republisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_tmp',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # 修改时间戳
        msg.header.stamp = self.get_clock().now().to_msg()

        # 发布消息到 /scan
        self.publisher_.publish(msg)
        self.get_logger().info('Republishing lidar data with updated timestamp')

def main(args=None):
    rclpy.init(args=args)
    lidar_republisher = LidarRePublisher()
    rclpy.spin(lidar_republisher)

    # 销毁节点
    lidar_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
