import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from pros_car_py.car_models import DeviceDataTypeEnum


class ArmController(Node):
    """
    A class to control a robotic arm.

    This class provides methods to update joint positions, reset the arm, and publish the current arm position.
    It handles individual and multiple joint updates with angle limits, and allows resetting the arm to default positions.

    Attributes:
        _init_joint_pos (list[float]): A list of initial joint angles (in radians).
        joint_pos (list[float]): A list of current joint angles (in radians).
        joint_trajectory_publisher_ (Publisher): ROS2 publisher for publishing the arm's joint trajectory.

    Methods:
        clamp(value, min_value, max_value):
            Clamps a value between a minimum and maximum limit.
        set_joint_position(joint_index, target_angle, lower_limit, upper_limit):
            Sets a specific joint to a target angle with specified limits.
        set_multiple_joint_positions(joint_positions):
            Sets multiple joints to target angles with specified limits.
        publish_arm_position():
            Publishes the current joint angles of the robotic arm.
        reset_arm():
            Resets all joints of the robotic arm to their default positions.
    """

    def __init__(self):
        """
        Initializes the ArmController node and sets default joint positions.

        The initial positions can also be set via environment variables, otherwise, the default values are used.
        """
        super().__init__("arm_controller")

        self._init_joint_pos = [
            math.radians(float(0)),  # Initial angle for Joint 0
            math.radians(float(0)),  # Initial angle for Joint 1
            math.radians(float(0)),  # Initial angle for Joint 2
            math.radians(float(0)),  # Initial angle for Joint 3
        ]
        self.joint_pos = self._init_joint_pos

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

    def clamp(self, value, min_value, max_value):
        """
        Clamps a value within a specified range.

        Args:
            value (float): The value to be clamped.
            min_value (float): The lower limit of the range.
            max_value (float): The upper limit of the range.

        Returns:
            float: The clamped value.

        Example:
            >>> self.clamp(5, 0, 10)
            5

            >>> self.clamp(15, 0, 10)
            10
        """
        return max(min_value, min(value, max_value))

    def set_joint_position(self, joint_index, target_angle, lower_limit, upper_limit):
        """
        Sets a specific joint to a target angle with specified limits.

        Args:
            joint_index (int): The index of the joint to update.
            target_angle (float): The target angle for the joint (in degrees).
            lower_limit (float): The lower limit for the joint's angle (in degrees).
            upper_limit (float): The upper limit for the joint's angle (in degrees).

        Example:
            Set Joint 0 to 30 degrees, within the range -90 to 90 degrees:
            >>> self.set_joint_position(0, 30, -90, 90)

            Set Joint 1 to -20 degrees, within the range -45 to 45 degrees:
            >>> self.set_joint_position(1, -20, -45, 45)
        """
        self.joint_pos[joint_index] = self.clamp(
            math.radians(target_angle),
            math.radians(lower_limit),
            math.radians(upper_limit),
        )

    def set_multiple_joint_positions(self, joint_positions):
        """
        Sets multiple joints to specific target angles.

        Args:
            joint_positions (list[tuple]): A list of tuples, each containing (joint_index, target_angle, lower_limit, upper_limit).

        Example:
            Set multiple joints to specific target positions:
            joint_positions = [
                (0, 30, -90, 90),  # Set Joint 0 to 30 degrees
                (1, -20, -45, 45), # Set Joint 1 to -20 degrees
                (2, 90, 0, 180)    # Set Joint 2 to 90 degrees
            ]
            >>> self.set_multiple_joint_positions(joint_positions)

        After setting, use `publish_arm_position()` to publish the current joint positions.
        """
        for joint_index, target_angle, lower_limit, upper_limit in joint_positions:
            self.set_joint_position(joint_index, target_angle, lower_limit, upper_limit)

    def publish_arm_position(self):
        """
        Publishes the current joint angles of the robotic arm.

        This method publishes the current positions of all joints, and is useful after updating the joint positions
        to notify other nodes of the current robotic arm's pose.

        Example:
            >>> self.publish_arm_position()
        """
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in self.joint_pos]
        msg.velocities = [0.0] * len(self.joint_pos)
        self.joint_trajectory_publisher_.publish(msg)

    def reset_arm(self):
        """
        Resets the robotic arm to the default position (all angles set to 0).

        This method resets the positions of all joints to their initial values, and the result can be
        published using the `publish_arm_position()` method.

        Example:
            >>> self.reset_arm()
            >>> self.publish_arm_position()
        """
        self.joint_pos = self._init_joint_pos
