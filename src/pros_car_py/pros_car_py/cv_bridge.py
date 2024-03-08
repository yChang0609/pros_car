import curses
import cv2
import numpy as np
import rclpy
import sys
import threading
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class NodeVideoReader(Node):
    """
    A class subscribes an ROS topic to read the compressed video and then show the video.

    Attributes
    ----------
    bridge : CvBridge
        The instance of the class CvBridge use to convert the ROS image to OpenCV image.
    subscription : Subscription
        The ROS subscription.
    video_writer
        The opencv video writer.
    cv_image: cv::Mat
        The opencv format image.

    Methods
    -------
    image_callback(msg: cv::Mat)
        The trigger function of the subscription.
        It reads the compressed image, converts to OpenCV format,
        shows the video and create video writer.
    init_video_writer(image: cv::Mat)
        Initialize the video writer.
    destroy_node()
        Destroy the ROS node.
    """

    # https://stackoverflow.com/questions/76537425/how-to-export-image-and-video-data-from-a-bag-file-in-ros2
    def __init__(self):
        super().__init__('image_to_video_converter')
        self.bridge = CvBridge()
        # Subscribe the compressed image.
        self.subscription = self.create_subscription(
            CompressedImage,
            '/out/compressed',
            self.image_callback,
            2147483647
        )
        # # Subscribe the raw image.
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera/color/image_raw',
        #     self.image_callback,
        #     2147483647
        # )
        self.video_writer = None
        self.cv_image = None

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            cv2.imshow('Robot Arm', self.cv_image)
            key = cv2.waitKey(1)
            if key == 27:  # esc
                cv2.destroyAllWindows()
                self.destroy_node()
                quit(0)

            # if self.video_writer is None:
            #     self.init_video_writer(self.cv_image)
            # self.video_writer.write(self.cv_image)
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: %s' % str(e))
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    # def init_video_writer(self, image):
    #     try:
    #         height, width, _ = image.shape
    #         video_format = 'mp4'  # or any other video format supported by OpenCV
    #         video_filename = 'output_video.' + video_format
    #         fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #         fps = 30  # Frames per second
    #         self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
    #     except Exception as e:
    #         self.get_logger().error('Error initializing video writer: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = NodeVideoReader()
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
