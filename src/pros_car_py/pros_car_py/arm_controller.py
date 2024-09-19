import os
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from pros_car_py.car_models import DeviceDataTypeEnum


class ArmController(Node):
    """控制機械手臂的類別"""

    def __init__(self):
        super().__init__("arm_controller")

        # 初始位置也可以用環境變數設定，否則使用預設值
        self.joint_pos = [
            math.radians(float(os.getenv("JOINT_0_INIT", 0))),  # Joint 0 初始角度
            math.radians(float(os.getenv("JOINT_1_INIT", 0))),  # Joint 1 初始角度
            math.radians(float(os.getenv("JOINT_2_INIT", 0))),  # Joint 2 初始角度
            math.radians(float(os.getenv("JOINT_3_INIT", 0))),  # Joint 3 初始角度
        ]

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

    def clamp(self, value, min_value, max_value):
        """限制值在一定範圍內"""
        return max(min_value, min(value, max_value))

    def update_joint_position(self, joint_index, delta_angle, lower_limit, upper_limit):
        """更新指定關節的角度"""
        self.joint_pos[joint_index] = self.clamp(
            self.joint_pos[joint_index] + delta_angle,
            math.radians(lower_limit),
            math.radians(upper_limit),
        )

    def update_multiple_joints(self, joint_updates):
        """
        一次更新多個關節的角度。
        joint_updates 是一個列表，每個元素包含 (joint_index, delta_angle, lower_limit, upper_limit)
        """
        for joint_index, delta_angle, lower_limit, upper_limit in joint_updates:
            self.update_joint_position(
                joint_index, delta_angle, lower_limit, upper_limit
            )

    def publish_arm_position(self):
        """發佈機械手臂的角度訊息"""
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in self.joint_pos]
        msg.velocities = [0.0] * len(self.joint_pos)
        self.joint_trajectory_publisher_.publish(msg)

    def reset_arm(self):
        """將機械手臂重置到預設位置"""
        self.joint_pos = [
            math.radians(float(os.getenv("JOINT_0_INIT", 0))),  # Joint 0 初始角度
            math.radians(float(os.getenv("JOINT_1_INIT", 0))),  # Joint 1 初始角度
            math.radians(float(os.getenv("JOINT_2_INIT", 0))),  # Joint 2 初始角度
            math.radians(float(os.getenv("JOINT_3_INIT", 0))),  # Joint 3 初始角度
        ]
