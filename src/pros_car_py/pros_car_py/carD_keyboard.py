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


class CarDKeyboardController(Node):
    def __init__(self, stdscr):
        super().__init__('car_D_keyboard')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            DeviceDataTypeEnum.car_D_state,
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            DeviceDataTypeEnum.car_D_control,  # topic name
            10
        )
        self.publisher_forward = self.create_publisher(
            String,
            "test",  # topic name
            10
        )

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
            "type": str(DeviceDataTypeEnum.car_D_control),
            "data": dict(CarDControl(
                target_vel=[self._vel1, self._vel2]
            ))
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)

        control_signal_forward = {
            "type": str(DeviceDataTypeEnum.car_D_control),
            "data": dict(CarDControl(
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

    def run(self):
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    if c == ord('w'):
                        self.handle_key_w()
                    elif c == ord('a'):
                        self.handle_key_a()
                    elif c == ord('s'):
                        self.handle_key_s()
                    elif c == ord('d'):
                        self.handle_key_d()
                    elif c == ord('e'):
                        self.handle_key_e()
                    elif c == ord('r'):
                        self.handle_key_r()
                    elif c == ord('z'):
                        self.handle_key_z()
                    elif c == ord('q'):  # Exit on 'q'
                        self._direction = 90  # degree
                        self._vel1 = 0  # rad/s
                        self._pub_control()

                        break
                    self._pub_control()
                    # self.pub_arm()
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

        # 
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

    def handle_key_a(self, vel: float = 10):
        # Your action for the 'a' key here
        self.stdscr.addstr(f"car go left")
        self._vel1 = vel  # rad/s
        self._vel2 = -vel  # rad/s
        self._vel3 = -vel  # rad/s
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

    def handle_key_d(self, vel: float = 10):
        self.stdscr.addstr(f"car go right")
        self._vel1 = -vel  # rad/s
        self._vel2 = vel  # rad/s
        self._vel3 = vel  # rad/s
        self._vel4 = -vel  # rad/s
        pass

    def handle_key_e(self, vel: float = 10):
        self.stdscr.addstr(f"car go clockwise")
        self._vel1 = vel  # rad/s
        self._vel2 = -vel  # rad/s
        self._vel3 = vel  # rad/s
        self._vel4 = -vel  # rad/s
        pass

    def handle_key_r(self, vel: float = 10):
        self.stdscr.addstr(f"car go counterclockwise")
        self._vel1 = -vel  # rad/s
        self._vel2 = vel  # rad/s
        self._vel3 = -vel  # rad/s
        self._vel4 = vel  # rad/s
        pass

    def handle_key_z(self):
        self.stdscr.addstr(f"stop car")
        self._vel1 = 0  # rad/s
        self._vel2 = 0
        self._vel3 = 0
        self._vel4 = 0
        pass


# ... Rest of your code, e.g. initializing rclpy and running the node

def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = CarDKeyboardController(stdscr)

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
