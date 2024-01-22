"""
PROS Arm Commercial License
Copyright 2023 帕亞科技 (PAIA-Tech) . All rights reserved.
Primary Author: 陳麒麟(Kylin Chen)
Contributor(s): 蘇文鈺(Alvin Su) ,
This software and associated documentation files are proprietary to 帕亞科技 (PAIA-Tech),
and are not to be copied, reproduced, or disclosed to any third party, in whole or in part, without
express prior written permission. Use and modification of this software is permitted for licensed
users only, and must always remain proprietary to 帕亞科技 (PAIA-Tech).


This code is a node to write control signal to specific arm.
It will receive topic /joint_trajectory_point from ROS and
transform to servo control signal for serial device.

"""

import orjson
import math
from .env import ARM_SERIAL_PORT_DEFAULT
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint

from serial import Serial


class ArmSerialWriter(Node):
    def __init__(self):
        super().__init__('arm_serial_writer')

        # Set up the serial connection
        serial_port = self.declare_parameter('serial_port', ARM_SERIAL_PORT_DEFAULT).value
        self._serial = Serial(serial_port, 115200, timeout=0)

        #  subscribe
        self._subscriber = self.create_subscription(
            JointTrajectoryPoint,
            'joint_trajectory_point',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: JointTrajectoryPoint):
        # TODO send pos to esp32
        radian_positions = msg.positions
        self.get_logger().info(f"receive {radian_positions}")

        # radian to degree
        degree_positions = [math.degrees(rad) % 360 for rad in radian_positions]
        ctrl_json = {"servo_target_angles": degree_positions}
        try:
            ctrl_str = orjson.dumps(ctrl_json, option=orjson.OPT_APPEND_NEWLINE)

            self._serial.write(ctrl_str)
        except orjson.JSONEncodeError as error:
            self.get_logger().error(f"Json encode error when recv message: {msg}")
            return
        # log
        self.get_logger().info(f"{ctrl_str}")


def main(args=None):
    rclpy.init(args=args)
    serial_writer = ArmSerialWriter()
    rclpy.spin(serial_writer)

    serial_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
