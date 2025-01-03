#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import socket
import json
import base64
import tempfile
import struct
import array

import OpenEXR
import Imath

from sensor_msgs.msg import Image

class UdpDepthEXRReceiverNode(Node):
    def __init__(self, udp_ip='0.0.0.0', udp_port=10000):
        super().__init__('udp_depth_exr_receiver')
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(Image, 'depth_image_topic', 10)

        # Init Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"UDP server listening on {self.udp_ip}:{self.udp_port}")

        # Timer to poll UDP
        self.timer = self.create_timer(0.01, self.udp_polling)

    def udp_polling(self):
        """Non-blocking recvfrom() to avoid blocking ROS spin."""
        self.sock.setblocking(False)
        try:
            data, addr = self.sock.recvfrom(65535)
            # self.get_logger().info(f"[DEBUG] Received {len(data)} bytes from {addr}")
        except BlockingIOError:
            return
        self.handle_udp_data(data, addr)

    def handle_udp_data(self, data, addr):
        """
        1) Parse JSON (timestamp + EXR Base64)
        2) Write EXR bytes to tempfile
        3) Use OpenEXR to read R channel (half float)
        4) Convert half float → 0~1 → mono16
        5) Publish as sensor_msgs/Image
        """
        try:
            msg_str = data.decode('utf-8')
            json_data = json.loads(msg_str)

            timestamp_ms = int(json_data['timestamp'])
            sec = timestamp_ms // 1000
            nanosec = (timestamp_ms % 1000) * 1_000_000

            # 1) Base64 decode → exr_bytes
            exr_bytes = base64.b64decode(json_data['data'])

            # 2) 寫到暫存檔，再以 OpenEXR.InputFile 開檔
            with tempfile.NamedTemporaryFile(suffix='.exr', delete=False) as tmp:
                tmp.write(exr_bytes)
                tmp.flush()
                filename = tmp.name

            # 3) 用 OpenEXR 讀取
            exr_file = OpenEXR.InputFile(filename)
            header = exr_file.header()
            dw = header['dataWindow']
            width = dw.max.x - dw.min.x + 1
            height = dw.max.y - dw.min.y + 1

            # 取出 channel, 假設我們只要 R channel
            # 如果 Unity 的深度存在哪個 channel，請自行對應 (可能是 R, A, 或同時 RGBA)
            pt_half = Imath.PixelType(Imath.PixelType.HALF)
            half_str = exr_file.channel('R', pt_half)
            exr_file.close()

            # 4) half float (2 bytes/pixel) → float32 → [0..1] → uint16 (mono16)
            pixel_count = len(half_str) // 2  # 每個 half = 2 bytes
            floats = array.array('f', [0]*pixel_count)

            for i in range(pixel_count):
                hb = half_str[2*i : 2*i+2]  # 2 bytes
                h_val = struct.unpack('<H', hb)[0]  # half bits (uint16)
                f_val = half_to_float(h_val)        # half → float32
                floats[i] = f_val

            # 將 float 值 (假設在 0~1) clamp & 轉成 0~65535
            depth_in_uint16 = array.array('H', [0]*pixel_count)
            for i in range(pixel_count):
                val_f = floats[i]
                # 視需求：這裡假設 0~1 depth，也可以擴大到其他量程
                val_clamped = max(0.0, min(1.0, val_f))
                depth_in_uint16[i] = int(val_clamped * 65535)

            # 5) 封裝成 ROS 2 Image (mono16)
            img_msg = Image()
            img_msg.header.stamp.sec = sec
            img_msg.header.stamp.nanosec = nanosec
            img_msg.header.frame_id = 'camera_depth_frame'
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = 'mono16'
            img_msg.is_bigendian = 0
            img_msg.step = 2 * width  # 2 bytes per pixel
            img_msg.data = depth_in_uint16.tobytes()

            self.publisher_.publish(img_msg)
            # self.get_logger().info(f"Received EXR: {width}x{height}, published mono16 image.")

        except Exception as e:
            self.get_logger().error(f"Error parsing EXR data: {e}")


def half_to_float(h):
    """
    將 16-bit half float (IEEE 754) 轉成 float32 的 bit manipulation。
    參考 https://gist.github.com/martinkallman/5049614
    """
    s = (h >> 15) & 0x00000001   # sign
    e = (h >> 10) & 0x0000001F   # exponent
    f =  h        & 0x000003FF   # fraction

    if e == 0:
        if f == 0:
            # zero
            return float(s << 31)
        else:
            # subnormal
            while (f & 0x00000400) == 0:
                f <<= 1
                e -= 1
            e += 1
            f &= ~0x00000400
    elif e == 31:
        if f == 0:
            # Inf
            return float((s << 31) | 0x7f800000)
        else:
            # NaN
            return float((s << 31) | 0x7f800000 | (f << 13))

    e = e + (127 - 15)
    f = f << 13
    bits = (s << 31) | (e << 23) | f
    fi = struct.pack('<I', bits)
    return struct.unpack('<f', fi)[0]


def main(args=None):
    rclpy.init(args=args)
    node = UdpDepthEXRReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
