import time
from rclpy.node import Node
import rclpy
from pros_car_py.car_models import *
from rclpy.duration import Duration
import orjson
from std_msgs.msg import String
import curses
import threading
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from pros_car_py.env import *
import math


class CarCKeyboardController(Node):
    def __init__(self, stdscr, vel: float = 10):
        super().__init__('car_C_keyboard')
        self.vel = vel
        self.rotate_angle = math.radians(10.0) # 控制機械手臂每次移動的角度
        self.rotate_speed = vel

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            DeviceDataTypeEnum.car_C_state,
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            DeviceDataTypeEnum.car_C_control,  # topic name
            10
        )
        self.publisher_forward = self.create_publisher(
            String,
            "test",  # topic name
            10
        )

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'joint_trajectory_point',
            10
        )
        self.joint_pos = [1.57, 0.0, 2.79, 0.87, 0.17]

        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        self._car_state_msg = ""
        self._vel1 = 0  # rad/s
        self._vel2 = 0
        self._vel3 = 0
        self._vel4 = 0

    def _sub_callback(self, msg):
        # Process the incoming message (if needed)
        # TODO show data in screen
        self._car_state_msg = str(self.get_clock().now()) + " " + msg.data

    def _pub_control(self):
        # Generate a random control signal
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_C_control),
            "data": dict(CarCControl(
                target_vel=[self._vel1, self._vel2]
            ))
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)

        control_signal_forward = {
            "type": str(DeviceDataTypeEnum.car_C_control),
            "data": dict(CarCControl(
                target_vel=[self._vel3, self._vel4]
            ))
        }
        # Convert the control signal to a JSON string
        control_msg_forward = String()
        control_msg_forward.data = orjson.dumps(control_signal_forward).decode()

        # Publish the control signal
        self.publisher_forward.publish(control_msg_forward)

        self.get_logger().info(f'publish {control_msg}')
        self.get_logger().info(f'publish to forward {control_msg_forward}')

    def run(self, vel=None):
        if vel is None:
            vel = self.vel
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    if c == ord('w'):
                        self.handle_key_w(vel)
                    elif c == ord('a'):
                        self.handle_key_a(vel)
                    elif c == ord('s'):
                        self.handle_key_s(vel)
                    elif c == ord('d'):
                        self.handle_key_d(vel)
                    elif c == ord('e'):
                        self.handle_key_e(vel)
                    elif c == ord('r'):
                        self.handle_key_r(vel)
                    elif c == ord('z'):
                        self.handle_key_z()
                    #  以下都是機械手臂
                    elif c == ord('i'):
                        self.handle_key_i()
                    elif c == ord('j'):
                        self.handle_key_j()
                    elif c == ord('k'):
                        self.handle_key_k()
                    elif c == ord('l'):
                        self.handle_key_l()
                    elif c == ord('u'):
                        self.handle_key_u()
                    elif c == ord('o'):
                        self.handle_key_o()
                    elif c == ord('y'):
                        self.handle_key_y()
                    elif c == ord('h'):
                        self.handle_key_h()
                    elif c == ord('n'):
                        self.handle_key_n()
                    elif c == ord('m'):
                        self.handle_key_m()
                    elif c == ord('b'):
                        self.handle_key_b()
                    elif c == ord('q'):  # Exit on 'q'
                        self._direction = 90  # degree
                        self._vel1 = 0  # rad/s
                        self._pub_control()
                        break
                    self._pub_control()
                    self.pub_arm()
                    print()
                else:
                    self.print_basic_info(ord(' '))
                    time.sleep(0.01)

            # origin_string = self.serial.readline()
            # self.stdscr.move(3, 0)
            # self.stdscr.addstr(f"{self.key_in_count:5d} receive: {origin_string} ")

        finally:
            curses.endwin()

    def print_basic_info(self, key):
        # Clear the screen
        self.stdscr.clear()

        self.stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

        # show receive data
        self.stdscr.move(1, 0)
        self.stdscr.addstr(f"Received msg : {self._car_state_msg}")

        # self.stdscr.move(4, 0)
        # self.stdscr.addstr(f"Arm pos : {self.joint_pos}")
        # self.stdscr.move(5, 0)

        # self.get_logger().debug(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

    def handle_key_w(self, vel: float = 10):
        # Your action for the 'w' key here
        self.stdscr.addstr(f"car go forward")

        self._vel1 = vel  # rad/s
        self._vel2 = vel  # rad/s
        self._vel3 = vel  # rad/s
        self._vel4 = vel  # rad/s
        # self.stdscr.move(1, 0)
        pass

    def handle_key_a(self, vel: float = 10, rate: float = 5):
        # Your action for the 'a' key here
        self.stdscr.addstr(f"car go left")
        self._vel1 = vel / rate  # rad/s
        self._vel2 = vel  # rad/s
        self._vel3 = vel / rate  # rad/s
        self._vel4 = vel  # rad/s

        pass

    # Add methods for other keys similarly
    def handle_key_s(self, vel: float = 10):
        self.stdscr.addstr(f"car go backward")
        self._vel1 = -vel  # rad/s
        self._vel2 = -vel  # rad/s
        self._vel3 = -vel  # rad/s
        self._vel4 = -vel  # rad/s
        pass

    def handle_key_d(self, vel: float = 10, rate: float = 5):
        self.stdscr.addstr(f"car go right")
        self._vel1 = vel  # rad/s
        self._vel2 = vel / rate  # rad/s
        self._vel3 = vel  # rad/s
        self._vel4 = vel / rate  # rad/s
        pass

    def handle_key_e(self, vel: float = 10):
        self.stdscr.addstr(f"car go clockwise")
        self._vel1 = -self.rotate_speed  # rad/s
        self._vel2 = self.rotate_speed  # rad/s
        self._vel3 = -self.rotate_speed  # rad/s
        self._vel4 = self.rotate_speed  # rad/s
        pass

    def handle_key_r(self, vel: float = 10):
        self.stdscr.addstr(f"car go counterclockwise")
        self._vel1 = self.rotate_speed  # rad/s
        self._vel2 = -self.rotate_speed  # rad/s
        self._vel3 = self.rotate_speed  # rad/s
        self._vel4 = -self.rotate_speed  # rad/s
        pass

    def handle_key_z(self):
        self.stdscr.addstr(f"stop car")
        self._vel1 = 0  # rad/s
        self._vel2 = 0
        self._vel3 = 0
        self._vel4 = 0
        pass

    #  機械手臂控制----------------
    def clamp(self, value, min_value, max_value):
        """限制值在一定範圍內"""
        return max(min_value, min(value, max_value))

    def handle_key_l(self):
        self.stdscr.addstr(f"arm turn right")
        # self.joint_pos[0] += self.rotate_angle
        self.joint_pos[0] = self.clamp(self.joint_pos[0] - self.rotate_angle, 0, math.radians(180))
        pass

    def handle_key_j(self):
        self.stdscr.addstr(f"arm turn left")
        # self.joint_pos[0] -= self.rotate_angle
        self.joint_pos[0] = self.clamp(self.joint_pos[0] + self.rotate_angle, 0, math.radians(180))
        pass

    def handle_key_i(self):
        self.stdscr.addstr(f"arm rift up")
        # self.joint_pos[1] += self.rotate_angle
        self.joint_pos[1] = self.clamp(self.joint_pos[1] - self.rotate_angle, math.radians(0), math.radians(180))
        pass

    def handle_key_k(self):
        self.stdscr.addstr(f"arm rift down")
        # self.joint_pos[1] -= self.rotate_angle
        self.joint_pos[1] = self.clamp(self.joint_pos[1] + self.rotate_angle, math.radians(0), math.radians(180))
        pass

    def handle_key_y(self):
        self.stdscr.addstr(f"arm catch!")
        # self.joint_pos[2] += self.rotate_angle
        self.joint_pos[2] = self.clamp(self.joint_pos[2] - self.rotate_angle, math.radians(0), math.radians(150))
        pass

    def handle_key_h(self):
        self.stdscr.addstr(f"arm release!")
        # self.joint_pos[2] -= self.rotate_angle
        self.joint_pos[2] = self.clamp(self.joint_pos[2] + self.rotate_angle, math.radians(0), math.radians(150))
        pass

    def handle_key_n(self):
        self.stdscr.addstr(f"arm release!")
        # self.joint_pos[3] -= self.rotate_angle
        self.joint_pos[3] = self.clamp(self.joint_pos[3] + self.rotate_angle, math.radians(50), math.radians(180))
        pass

    def handle_key_m(self):
        self.stdscr.addstr(f"arm release!")
        # self.joint_pos[3] += self.rotate_angle
        self.joint_pos[3] = self.clamp(self.joint_pos[3] - self.rotate_angle, math.radians(50), math.radians(180))
        pass

    def handle_key_u(self):
        self.stdscr.addstr(f"arm j4 rotate left")
        # self.joint_pos[4] -= self.rotate_angle
        self.joint_pos[4] = self.clamp(self.joint_pos[4] + self.rotate_angle, math.radians(0), math.radians(70))
        pass

    def handle_key_o(self):
        self.stdscr.addstr(f"arm j4 rotate right")
        # self.joint_pos[4] += self.rotate_angle
        self.joint_pos[4] = self.clamp(self.joint_pos[4] - self.rotate_angle, math.radians(0), math.radians(70))
        pass

    def handle_key_b(self):
        # 初始化機器手臂到預設位置的方法
        self.stdscr.addstr(f"將機器手臂初始化到預設位置...")
        # self.joint_pos = [0.0, 1.57, 1.57, 0.52, 1.22]  # 以弧度表示的角度（0, 90, 90, 30, 70 度）
        # self.pub_arm()
        self.joint_pos = [math.radians(90), math.radians(0), math.radians(160), math.radians(50), math.radians(10)]


    def pub_arm(self):
        msg = JointTrajectoryPoint()
        # msg.positions = [math.degrees(pos) for pos in self.joint_pos]   # Replace with actual desired positions
        msg.positions = [float(pos) for pos in self.joint_pos]
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
        self.joint_trajectory_publisher_.publish(msg)


# ... Rest of your code, e.g. initializing rclpy and running the node
def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    vel = 10 if WHEEL_SPEED is None else float(WHEEL_SPEED)
    node = CarCKeyboardController(stdscr, vel=vel)

    # Spin the node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped


if __name__ == '__main__':
    main()
