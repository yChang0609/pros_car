import rclpy
import time
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from argparse import ArgumentParser


class ArmTestNode(Node):
    """測試機械手臂的節點"""

    def __init__(self, joint_count):
        super().__init__("arm_test_node")
        self.joint_count = joint_count  # 軀幹數量
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, "robot_arm", 10
        )

    def move_joints(self):
        """讓每個軀幹從 0 度到 180 度發送訊息，並間隔 0.5 秒"""
        for angle in range(0, 181, 10):  # 0度到180度，步進10度
            msg = JointTrajectoryPoint()
            msg.positions = [
                math.radians(angle)
            ] * self.joint_count  # 所有關節設置為相同的角度
            msg.velocities = [0.0] * self.joint_count  # 初始化速度為 0
            self.joint_trajectory_publisher_.publish(msg)
            self.get_logger().info(
                f"Published joint angles: {angle} degrees to {self.joint_count} joints."
            )
            time.sleep(0.5)  # 每次間隔 0.5 秒


def main(args=None):
    rclpy.init(args=args)

    # 使用 argparse 來解析軀幹數量參數
    parser = ArgumentParser(
        description="Test arm controller by sending joint positions."
    )
    parser.add_argument(
        "--joints", type=int, default=4, help="Number of joints to control (default: 4)"
    )
    args = parser.parse_args()

    # 初始化節點
    arm_test_node = ArmTestNode(joint_count=args.joints)

    try:
        arm_test_node.move_joints()  # 開始動作測試
    except KeyboardInterrupt:
        pass

    # 結束時關閉 rclpy
    arm_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
