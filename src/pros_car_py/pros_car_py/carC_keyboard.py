import threading
import curses
import rclpy
from pros_car_py.car_controller import CarController
from pros_car_py.arm_controller import ArmController
from pros_car_py.keyboard_controller import KeyboardController


def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()

    car_controller = CarController()
    arm_controller = ArmController()

    # 使用環境變數 WHEEL_SPEED，若無則使用預設速度 10
    keyboard_controller = KeyboardController(
        stdscr, car_controller, arm_controller, default_vel=10
    )

    # Spin the ROS node in a separate thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(car_controller,), daemon=True
    )
    spin_thread.start()

    try:
        keyboard_controller.run()
    finally:
        curses.endwin()
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped


if __name__ == "__main__":
    main()
