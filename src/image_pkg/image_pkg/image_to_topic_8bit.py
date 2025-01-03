import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket
import json
import base64
import numpy as np
from datetime import datetime
import cv2


def decode_depth_data(message):
    """
    解碼從 Unity 接收到的深度數據，並還原為深度影像。
    """
    try:
        data = json.loads(message)
        timestamp = data.get("timestamp", "")
        encoded_data = data.get("data", "")
        raw_data = base64.b64decode(encoded_data)

        # 使用 OpenCV 解碼 PNG 資料
        png_array = np.frombuffer(raw_data, dtype=np.uint8)
        depth_image = cv2.imdecode(png_array, cv2.IMREAD_UNCHANGED)

        return timestamp, depth_image
    except Exception as e:
        print(f"Error decoding depth data: {e}")
        return None, None


class DepthDataReceiver(Node):
    def __init__(self):
        super().__init__('depth_data_receiver')

        # 初始化 UDP Socket
        self.udp_ip = '0.0.0.0'
        self.udp_port = 10000
        self.buffer_size = 65536

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"Listening for UDP data on {self.udp_ip}:{self.udp_port}")

        # 创建 ROS 话题发布者
        self.publisher_ = self.create_publisher(Image, 'camera/depth/image_raw', 10)

        # 数据统计
        self.received_count = 0
        self.last_received_time = None

        # 创建一个定时器来处理 UDP 接收
        self.timer = self.create_timer(0.05, self.receive_and_publish)

        # 创建一个定时器定期打印统计信息
        self.monitor_timer = self.create_timer(5.0, self.print_statistics)

    def receive_and_publish(self):
        try:
            message, _ = self.sock.recvfrom(self.buffer_size)
            timestamp, depth_image = decode_depth_data(message.decode('utf-8'))

            if depth_image is not None:
                # 更新统计信息
                self.received_count += 1
                self.last_received_time = datetime.now()

                # 构造 ROS 影像消息
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera"
                msg.height = depth_image.shape[0]  # 高度应与 Unity 的深度图一致
                msg.width = depth_image.shape[1]   # 宽度应与 Unity 的深度图一致
                msg.encoding = "16UC1" if depth_image.dtype == np.uint16 else "8UC1"
                # msg.encoding = "mono16"
                msg.is_bigendian = 0
                msg.step = depth_image.shape[1] * depth_image.dtype.itemsize
                msg.data = depth_image.tobytes()

                # 发布消息
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published depth data at timestamp {timestamp}")

        except Exception as e:
            self.get_logger().error(f"Error receiving or processing UDP data: {e}")

    def print_statistics(self):
        if self.last_received_time:
            time_since_last = (datetime.now() - self.last_received_time).total_seconds()
            self.get_logger().info(
                f"Received {self.received_count} messages. "
                f"Last message received {time_since_last:.2f} seconds ago."
            )
        else:
            self.get_logger().info("No messages received yet.")


def main(args=None):
    rclpy.init(args=args)
    receiver = DepthDataReceiver()

    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
