import curses
import cv2
import numpy as np
import rclpy
import sys
import threading
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class ImageToVideoConverter(Node):
    # https://stackoverflow.com/questions/76537425/how-to-export-image-and-video-data-from-a-bag-file-in-ros2
    def __init__(self):
        super().__init__('image_to_video_converter')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/out/compressed',
            self.image_callback,
            2147483647
        )
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera/color/image_raw',
        #     self.image_callback,
        #     2147483647
        # )
        self.video_writer = None
        self.cv_image = None
        print("FINISH constructor", file=sys.stderr)

    def image_callback(self, msg):
        print("REACH image callback", file=sys.stderr)
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # print(self.cv_image, file=sys.stderr)

            cv2.imshow('Robot Arm', self.cv_image)
            key = cv2.waitKey(1)

            if self.video_writer is None:
                self.init_video_writer(self.cv_image)
            self.video_writer.write(self.cv_image)
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: %s' % str(e))
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_format = 'mp4'  # or any other video format supported by OpenCV
            video_filename = 'output_video.' + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 30  # Frames per second
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error('Error initializing video writer: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()

    def display_video(self):
        if self.cv_image is None:
            print("Error, there is no image.", file=sys.stderr)
        # while True:
        #     cv2.imshow('Robot Arm', self.cv_image)
        #     key = cv2.waitKey(1)
        #     if key == 27:  # esc
        #         cv2.destroyAllWindows()
        #         break


def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = ImageToVideoConverter()
    """
    Note:
    - If you use multithreading, you will encounter sync problem.
        That is, the image is OK in constructor, but is None in other threads.
    """
    # # Spin the node in a separate thread
    # spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # spin_thread.start()
    #
    # try:
    #     node.display_video()
    # finally:
    #     curses.endwin()
    #     node.get_logger().info(f'Quit keyboard!')
    #     rclpy.shutdown()
    #     spin_thread.join()  # Ensure the spin thread is cleanly stopped
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
