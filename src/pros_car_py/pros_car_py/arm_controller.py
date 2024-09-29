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
            math.radians(float(0)),  # Joint 0 初始角度
            math.radians(float(0)),  # Joint 1 初始角度
            math.radians(float(0)),  # Joint 2 初始角度
            math.radians(float(0)),  # Joint 3 初始角度
        ]

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

    def clamp(self, value, min_value, max_value):
        """限制值在一定範圍內

        範例:
        >>> self.clamp(5, 0, 10)
        5

        >>> self.clamp(15, 0, 10)
        10
        """
        return max(min_value, min(value, max_value))

    def update_joint_position(self, joint_index, delta_angle, lower_limit, upper_limit):
        """更新單一關節，基於目前關節角度做 delta_angle 的關節加減

        範例:
        將 Joint 0 的角度增加 10 度，範圍在 -90 度到 90 度之間:
        >>> self.update_joint_position(0, math.radians(10), -90, 90)

        將 Joint 1 的角度減少 15 度，範圍在 -45 度到 45 度之間:
        >>> self.update_joint_position(1, math.radians(-15), -45, 45)
        """
        self.joint_pos[joint_index] = self.clamp(
            self.joint_pos[joint_index] + delta_angle,
            math.radians(lower_limit),
            math.radians(upper_limit),
        )

    def update_multiple_joints(self, joint_updates):
        """
        一次更新多個關節的角度。
        joint_updates 是一個列表，每個元素包含 (joint_index, delta_angle, lower_limit, upper_limit)

        範例:
        一次更新多個關節的角度:
        joint_updates = [
            (0, math.radians(10), -90, 90),  # 將 Joint 0 增加 10 度
            (1, math.radians(-15), -45, 45), # 將 Joint 1 減少 15 度
            (2, math.radians(20), 0, 180)    # 將 Joint 2 增加 20 度
        ]
        >>> self.update_multiple_joints(joint_updates)
        """
        for joint_index, delta_angle, lower_limit, upper_limit in joint_updates:
            self.update_joint_position(
                joint_index, delta_angle, lower_limit, upper_limit
            )

    def publish_arm_position(self):
        """發佈機械手臂的角度訊息

        範例:
        >>> self.publish_arm_position()

        說明:
        此方法會將當前所有關節的角度位置發佈出去，適合在調整關節位置後調用，以通知其他節點當前的機械手臂姿態。
        """
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in self.joint_pos]
        msg.velocities = [0.0] * len(self.joint_pos)
        self.joint_trajectory_publisher_.publish(msg)

    def reset_arm(self):
        """將機械手臂重置到預設位置

        範例:
        >>> self.reset_arm()
        >>> self.publish_arm_position()

        說明:
        此方法會將所有關節的位置重置為初始值（0 度），然後可以使用 `publish_arm_position` 將重置的結果發佈出去。
        """
        self.joint_pos = [
            math.radians(float(0)),  # Joint 0 初始角度
            math.radians(float(0)),  # Joint 1 初始角度
            math.radians(float(0)),  # Joint 2 初始角度
            math.radians(float(0)),  # Joint 3 初始角度
        ]

    def set_joint_position(self, joint_index, target_angle, lower_limit, upper_limit):
        """設置指定單一關節到目標角度

        範例:
        將 Joint 0 設置到 30 度，範圍在 -90 到 90 度之間:
        >>> self.set_joint_position(0, 30, -90, 90)

        將 Joint 1 設置到 -20 度，範圍在 -45 到 45 度之間:
        >>> self.set_joint_position(1, -20, -45, 45)
        """
        self.joint_pos[joint_index] = self.clamp(
            math.radians(target_angle),
            math.radians(lower_limit),
            math.radians(upper_limit),
        )

    def set_multiple_joint_positions(self, joint_positions):
        """
        一次設置多個關節到具體目標角度。
        joint_positions 是一個列表，每個元素包含 (joint_index, target_angle, lower_limit, upper_limit)

        範例:
        一次設置多個關節的位置:
        joint_positions = [
            (0, 30, -90, 90),  # 將 Joint 0 設置到 30 度
            (1, -20, -45, 45), # 將 Joint 1 設置到 -20 度
            (2, 90, 0, 180)    # 將 Joint 2 設置到 90 度
        ]
        >>> self.set_multiple_joint_positions(joint_positions)

        說明:
        設置完成後，可以調用 `publish_arm_position()` 發佈當前關節的位置。
        """
        for joint_index, target_angle, lower_limit, upper_limit in joint_positions:
            self.set_joint_position(joint_index, target_angle, lower_limit, upper_limit)
