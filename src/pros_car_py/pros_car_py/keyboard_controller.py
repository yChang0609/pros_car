import os
import curses
import math


class KeyboardController:
    """鍵盤控制邏輯"""

    def __init__(self, stdscr, car_controller, arm_controller, default_vel=10):
        self.stdscr = stdscr
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        self.key_in_count = 0
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)

        # 使用環境變數 WHEEL_SPEED，若無則使用 default_vel

        self.vel = float(os.getenv("WHEEL_SPEED", default_vel))
        self.rotate_angle = math.radians(float(os.getenv("ARM_ROTATE_ANGLE", 10.0)))
        joint_updates = [
            (
                0,
                math.radians(10),
                0,
                3600,
            ),  # 將 Joint 0 增加 10 度，範圍在 -90 度到 90 度之間
            (
                1,
                math.radians(10),
                0,
                3600,
            ),  # 將 Joint 1 減少 15 度，範圍在 -45 度到 45 度之間
            (
                2,
                math.radians(10),
                0,
                3600,
            ),  # 將 Joint 2 增加 20 度，範圍在 0 度到 180 度之間
            (
                3,
                math.radians(10),
                0,
                3600,
            ),
        ]
        joint_updates2 = [
            (
                0,
                math.radians(-10),
                0,
                3600,
            ),  # 將 Joint 0 增加 10 度，範圍在 -90 度到 90 度之間
            (
                1,
                math.radians(-10),
                0,
                3600,
            ),  # 將 Joint 1 減少 15 度，範圍在 -45 度到 45 度之間
            (
                2,
                math.radians(-10),
                0,
                3600,
            ),  # 將 Joint 2 增加 20 度，範圍在 0 度到 180 度之間
            (
                3,
                math.radians(-10),
                0,
                3600,
            ),
        ]
        self.key_map = {
            "w": self.handle_car_forward,
            "a": self.handle_car_left,
            "s": self.handle_car_backward,
            "d": self.handle_car_right,
            "e": self.handle_car_rotate_cw,
            "r": self.handle_car_rotate_ccw,
            "z": self.handle_car_stop,
            "i": lambda: self.arm_controller.update_multiple_joints(joint_updates2),
            "k": lambda: self.arm_controller.update_multiple_joints(joint_updates),
            "b": lambda: self.arm_controller.reset_arm(),
        }
        self.last_key = None  # 儲存上一次按鍵輸入

    def handle_key_input(self, c):
        """處理鍵盤輸入"""
        if c in self.key_map:
            self.key_map[c]()
            self.car_controller.publish_control()
            self.arm_controller.publish_arm_position()

        # 記錄當前輸入的按鍵，準備在下一次更新畫面時顯示
        self.last_key = c

    def display_info(self):
        """顯示最新的按鍵資料，直到下一個按鍵輸入"""
        if self.last_key:
            self.stdscr.clear()  # 清空終端畫面
            self.stdscr.addstr(
                0, 0, f"Key pressed: {self.last_key}"
            )  # 顯示最後按下的按鍵
            self.stdscr.refresh()  # 刷新終端顯示最新內容

    def handle_car_forward(self):
        """處理車輛前進"""
        self.car_controller.update_velocity(self.vel, self.vel, self.vel, self.vel)

    def handle_car_left(self):
        """處理車輛左轉"""
        self.car_controller.update_velocity(
            self.vel / 5, self.vel, self.vel / 5, self.vel
        )

    def handle_car_backward(self):
        """處理車輛後退"""
        self.car_controller.update_velocity(-self.vel, -self.vel, -self.vel, -self.vel)

    def handle_car_right(self):
        """處理車輛右轉"""
        self.car_controller.update_velocity(
            self.vel, self.vel / 5, self.vel, self.vel / 5
        )

    def handle_car_rotate_cw(self):
        """處理車輛順時針旋轉"""
        self.car_controller.update_velocity(-self.vel, self.vel, -self.vel, self.vel)

    def handle_car_rotate_ccw(self):
        """處理車輛逆時針旋轉"""
        self.car_controller.update_velocity(self.vel, -self.vel, self.vel, -self.vel)

    def handle_car_stop(self):
        """停止車輛"""
        self.car_controller.update_velocity(0, 0, 0, 0)

    def run(self):
        """運行鍵盤控制邏輯"""
        self.stdscr.nodelay(False)  # 設置為阻塞模式，等待輸入
        while True:
            self.display_info()  # 顯示當前的按鍵
            c = self.stdscr.getch()  # 阻塞等待按鍵輸入
            if c == ord("q"):
                break
            if c != curses.ERR:
                self.handle_key_input(chr(c))
