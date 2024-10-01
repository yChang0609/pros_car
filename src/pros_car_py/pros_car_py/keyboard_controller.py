import os
import curses
import threading
import rclpy
from pros_car_py.joint_config import JOINT_UPDATES_POSITIVE, JOINT_UPDATES_NEGATIVE
from pros_car_py.car_controller import CarController
from pros_car_py.arm_controller import ArmController


class KeyboardController:
    """鍵盤控制邏輯，專注於定義按鍵與控制行為的對應"""

    def __init__(self, stdscr, car_controller, arm_controller, default_vel=10):
        self.stdscr = stdscr
        self.last_key = None
        self.car_controller = car_controller
        self.arm_controller = arm_controller

        self.vel = float(os.getenv("WHEEL_SPEED", default_vel))
        self.key_map = self.init_key_map()

        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)

    def init_key_map(self):
        """初始化按鍵映射，定義按鍵與動作的對應"""
        return {
            "w": lambda: self.car_controller.move_forward(self.vel),
            "a": lambda: self.car_controller.turn_left(self.vel),
            "s": lambda: self.car_controller.move_backward(self.vel),
            "d": lambda: self.car_controller.turn_right(self.vel),
            "e": lambda: self.car_controller.rotate_cw(self.vel),
            "r": lambda: self.car_controller.rotate_ccw(self.vel),
            "z": lambda: self.car_controller.stop(),
            "i": lambda: self.arm_controller.update_multiple_joints(JOINT_UPDATES_POSITIVE),
            "k": lambda: self.arm_controller.update_multiple_joints(JOINT_UPDATES_NEGATIVE),
            "b": lambda: self.arm_controller.reset_arm(),
        }

    def run(self):
        """運行鍵盤控制邏輯"""
        self.stdscr.nodelay(False)
        while True:
            self.display_info()
            c = self.stdscr.getch()
            if c == ord("q"):
                break
            self.handle_key_input(chr(c))

    def handle_key_input(self, c):
        """處理鍵盤輸入"""
        if c in self.key_map:
            self.key_map[c]()
            self.car_controller.publish_control()
            self.arm_controller.publish_arm_position()
        self.last_key = c

    def display_info(self):
        """顯示最新的按鍵資料"""
        if self.last_key:
            self.stdscr.clear()
            self.stdscr.addstr(0, 0, f"Key pressed: {self.last_key}")
            self.stdscr.refresh()


def main():
    rclpy.init()
    stdscr = curses.initscr()

    car_controller = CarController()
    arm_controller = ArmController()
    keyboard_controller = KeyboardController(stdscr, car_controller, arm_controller, default_vel=10)

    try:
        keyboard_controller.run()
    finally:
        curses.endwin()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
