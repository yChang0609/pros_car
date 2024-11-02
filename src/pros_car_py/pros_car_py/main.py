import os
import curses
import threading
import rclpy
import time
import io
import sys
from pros_car_py.joint_config import JOINT_UPDATES_POSITIVE, JOINT_UPDATES_NEGATIVE
from pros_car_py.car_controller import CarController
from pros_car_py.arm_controller import ArmController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.ros_communicator import RosCommunicator
from pros_car_py.custom_control import CustomControl
from pros_car_py.ik_solver import RobotIKSolver
import logging
class KeyboardController:
    """鍵盤控制邏輯，專注於定義按鍵與控制行為的對應"""

    def __init__(self, stdscr, car_controller, arm_controller, custom_control, default_vel=10):
        self.stdscr = stdscr
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        self.custom_control = custom_control
        self.vel = float(os.getenv("WHEEL_SPEED", default_vel))
        self.mode = None
        self.output_buffer = io.StringIO()
        self.last_key = None



        # 初始化 curses
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        
        # 初始化顏色
        curses.start_color()
        curses.init_pair(1, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)

    def run_mode(self):
        """在特定模式下運行鍵盤控制邏輯"""
        self.stdscr.nodelay(True)
        last_mode = None
        
        while True:
            c = self.stdscr.getch()
            
            if self.process_auto_mode_input(c):
                break
                
            self.process_manual_mode_input(c)
            
            # 更新顯示
            if self._should_update_display(last_mode, c):
                self._update_display()
                last_mode = self.mode
                
            time.sleep(0.1)
        
        self._reset_mode()

    def process_auto_mode_input(self, c):
        """處理自動模式的輸入"""
        # 定義控制器映射
        controller_map = {
            "Auto Nav": self.car_controller,
            "Auto Arm Control": self.arm_controller
        }
        
        # 獲取當前模式的控制器
        controller = controller_map.get(self.mode)
        if not controller:
            return False

        # 處理退出命令
        if c == ord("q"):
            controller.auto_control(key='q')
            self.stdscr.clear()
            return True
        
        # 處理一般自動控制
        controller.auto_control()
        return False

    def process_manual_mode_input(self, c):
        """處理不同模式下的輸入"""
        if c != -1:  # 有按鍵輸入時
            self.handle_key_input(chr(c))

    def _should_update_display(self, last_mode, c):
        """判斷是否需要更新顯示"""
        return self.mode != last_mode or c != -1

    def _update_display(self):
        """更新顯示內容"""
        self.stdscr.clear()
        self.display_mode_info()
        self.stdscr.refresh()

    def _reset_mode(self):
        """重置模式相關的狀態"""
        self.mode = None
        self.output_buffer = io.StringIO()
        self.last_key = None

    def display_mode_info(self):
        """顯示當前模式的信息和控制說明"""
        self.stdscr.clear()
        
        # 顯示當前模式
        self.stdscr.attron(curses.color_pair(1) | curses.A_BOLD)
        self.stdscr.addstr(0, 0, f"Current Mode: {self.mode}")
        self.stdscr.attroff(curses.color_pair(1) | curses.A_BOLD)

        # 顯示控制說明
        self.stdscr.attron(curses.color_pair(2))
        self.stdscr.addstr(1, 0, "Controls:")
        self.stdscr.attroff(curses.color_pair(2))

        # 根據模式顯示不同的控制說明
        if self.mode == "Car Control":
            self.stdscr.addstr(2, 0, "Use arrow keys to control the car", curses.color_pair(3))
        elif self.mode == "Arm Control":
            self.stdscr.addstr(2, 0, "Use number keys to control arm joints", curses.color_pair(3))
        elif self.mode == "Custom Control":
            self.stdscr.addstr(2, 0, "Use custom control keys", curses.color_pair(3))
        elif self.mode in ["Auto Nav", "Auto Arm Control"]:
            self.stdscr.addstr(2, 0, "Running in automatic mode", curses.color_pair(3))
            self.stdscr.addstr(3, 0, "Internal print output:", curses.color_pair(2))
            
            # 顯示最新的10條訊息
            lines = self.output_buffer.getvalue().split('\n')
            latest_lines = lines[-10:]  # 取最新的10條
            y = 4
            for line in latest_lines:
                self.stdscr.addstr(y, 0, line[:curses.COLS-1])
                y += 1
                if y >= curses.LINES - 1:
                    break
            
            # 清理舊訊息
            self.output_buffer = io.StringIO("\n".join(latest_lines))

        self.stdscr.addstr(curses.LINES - 1, 0, "Press 'q' to return to main menu", curses.color_pair(4))
        
        if self.last_key:
            self.stdscr.addstr(6, 0, f"Last key pressed: {self.last_key}")
        
        self.stdscr.refresh()

    def handle_key_input(self, c):
        """處理鍵盤輸入"""
        old_stdout = sys.stdout
        sys.stdout = self.output_buffer
        try:
            if self.mode == "Car Control":
                self.car_controller.manual_control(c)
            elif self.mode == "Arm Control":
                self.arm_controller.manual_control(c)
            elif self.mode == "Auto Nav":
                self.car_controller.auto_control(key=c)
            elif self.mode == "Auto Arm Control":
                self.arm_controller.auto_control(key=c)
            elif self.mode == "Custom Control":
                self.custom_control.manual_control(c)
            self.last_key = c
        finally:
            sys.stdout = old_stdout
        self.display_mode_info()

def init_ros_node():
    rclpy.init()
    node = RosCommunicator()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread

def main():
    stdscr = curses.initscr()
    ros_communicator, ros_thread = init_ros_node()
    data_processor = DataProcessor(ros_communicator)
    nav2_processing = Nav2Processing(ros_communicator, data_processor)
    car_controller = CarController(ros_communicator, nav2_processing)
    ik_solver = RobotIKSolver()
    arm_controller = ArmController(ros_communicator, nav2_processing, ik_solver, num_joints=5)
    custom_control = CustomControl(car_controller, arm_controller)
    keyboard_controller = KeyboardController(stdscr, car_controller, arm_controller, custom_control, default_vel=10)
    
    try:
        while True:
            # stdscr.clear()
            stdscr.attron(curses.color_pair(1) | curses.A_BOLD)
            stdscr.addstr(0, 0, "Select a mode:")
            stdscr.attroff(curses.color_pair(1) | curses.A_BOLD)
            
            stdscr.addstr(1, 0, "1. Car Control", curses.color_pair(3))
            stdscr.addstr(2, 0, "2. Arm Control", curses.color_pair(3))
            stdscr.addstr(3, 0, "3. Auto Nav", curses.color_pair(2))
            stdscr.addstr(4, 0, "4. Auto Arm Control", curses.color_pair(2))
            stdscr.addstr(5, 0, "5. Custom Control", curses.color_pair(4))
            stdscr.addstr(6, 0, "q. Quit", curses.color_pair(4))
            # stdscr.refresh()  # 只在這裡刷新畫面

            choice = stdscr.getch()  # 等待使用者輸入
            if choice == ord('q'):
                break
            elif choice in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                mode = {ord('1'): "Car Control", ord('2'): "Arm Control", ord('3'): "Auto Nav", ord('4'): "Auto Arm Control", ord('5'): "Custom Control"}[choice]
                keyboard_controller.mode = mode
                keyboard_controller.output_buffer = io.StringIO()
                keyboard_controller.run_mode()
    finally:
        curses.endwin()
        rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
