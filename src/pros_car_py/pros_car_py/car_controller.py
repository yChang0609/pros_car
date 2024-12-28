from rclpy.node import Node
from std_msgs.msg import String
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl
import threading
import time

class CarController():

    def __init__(self, ros_communicator, nav_processing):
        self.ros_communicator = ros_communicator
        self.nav_processing = nav_processing      
        # 用來管理後台執行緒的屬性
        self._auto_nav_thread = None
        self._stop_event = None
        self._thread_running = False
        self.flag = 0

        self._auto_nav_thread = None
        self._stop_event = threading.Event()
        self._thread_running = False

    def update_action(self, action_key):
        """
        Updates the velocity for each of the car's wheels.

        Args:
            vel1 (float): Velocity for the rear left wheel (rad/s).
            vel2 (float): Velocity for the rear right wheel (rad/s).
            vel3 (float): Velocity for the front left wheel (rad/s).
            vel4 (float): Velocity for the front right wheel (rad/s).

        Example:
            car_controller.update_velocity(10, 10, 10, 10)  # Set all wheels' velocity to 10 rad/s.
        """
        self.ros_communicator.publish_car_control(action_key)

    def manual_control(self, key):
        """
        Controls the car based on single character inputs ('w', 'a', 's', 'd', 'z').

        Args:
            key (str): A single character representing a control command.
                'w' - move forward
                's' - move backward
                'a' - turn left
                'd' - turn right
                'z' - stop

        Example:
            car_controller.manual_control('w')  # Moves the car forward.
        """
        if key == 'w':
            self.update_action("FORWARD")
        elif key == 's':
            self.update_action("BACKWARD")
        elif key == 'a':
            self.update_action("LEFT_FRONT")
        elif key == 'd':
            self.update_action("RIGHT_FRONT")
        elif key == 'e':
            self.update_action("COUNTERCLOCKWISE_ROTATION")
        elif key == 'r':
            self.update_action("CLOCKWISE_ROTATION")
        elif key == 'z':
            self.update_action("STOP")
        elif key == 'q':
            self.update_action("STOP")
            return True
        else:
            pass

    def auto_control(self, mode="manual_auto_nav", target=None, key=None):
        """
        自動控制邏輯
        Args:
            mode: 控制模式 ("auto_nav" 或 "manual_nav")
            target: 目標座標 (用於 manual_nav 模式)
            key: 鍵盤輸入
        """
        # 如果有按鍵輸入
        if self.flag == 0:
            stop_event = threading.Event()
            thread = threading.Thread(target=self.background_task, args=(stop_event,))

        if key == "q":
            # 按下 q 時停止導航並退出
            if self._thread_running:
                self._stop_event.set()
                self._auto_nav_thread.join()
                self._thread_running = False

            action_key = self.nav_processing.stop_nav()
            self.ros_communicator.publish_car_control(action_key, publish_rear=True, publish_front=True)
            return True
        
        if not self._thread_running:
            self._stop_event.clear()  # 清除之前的停止狀態
            self._auto_nav_thread = threading.Thread(
                target=self.background_task,
                args=(self._stop_event, mode, target),
                daemon=True
            )
            self._auto_nav_thread.start()
            self._thread_running = True

        return False
            
    def background_task(self, stop_event, mode, target):
        """
        後台任務：不斷執行導航動作直到 stop_event 被設定。
        """
        while not stop_event.is_set():
            if mode == "manual_auto_nav":
                action_key = self.nav_processing.get_action_from_nav2_plan(goal_coordinates=None)
            elif mode == "target_auto_nav":
                action_key = self.nav_processing.get_action_from_nav2_plan(goal_coordinates=target)
            
            # 發布控制指令
            self.ros_communicator.publish_car_control(action_key, publish_rear=True, publish_front=True)
            
            # 模擬導航延遲
            time.sleep(0.1)

        # 收尾動作
        print("[background_task] Navigation stopped.")

