from rclpy.node import Node
from std_msgs.msg import String
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl


class CarController():
    """
    A class to control the car.

    This class provides methods to update wheel velocities and publish the control signals for the car's movement.
    It handles different movements such as forward, backward, turning, and rotating the car, with the ability
    to control rear and front wheels separately.

    Attributes:
        vel (float): Default velocity for the car's wheels.
        _vel1 (float): Velocity for rear left wheel (rad/s).
        _vel2 (float): Velocity for rear right wheel (rad/s).
        _vel3 (float): Velocity for front left wheel (rad/s).
        _vel4 (float): Velocity for front right wheel (rad/s).

    Methods:
        update_velocity(vel1, vel2, vel3, vel4):
            Updates the velocity for each of the car's wheels.
        publish_control(publish_rear=True, publish_front=True):
            Publishes the control signals for the car's wheels.
        move_forward(vel):
            Moves the car forward by setting all wheels to the given velocity.
        move_backward(vel):
            Moves the car backward by setting all wheels to the negative of the given velocity.
        turn_left(vel):
            Turns the car left by reducing the velocity of the left wheels.
        turn_right(vel):
            Turns the car right by reducing the velocity of the right wheels.
        rotate_cw(vel):
            Rotates the car clockwise by setting opposite velocities for the left and right wheels.
        rotate_ccw(vel):
            Rotates the car counterclockwise by setting opposite velocities for the left and right wheels.
        stop():
            Stops the car by setting all wheels' velocities to zero.
    """

    def __init__(self, ros_communicator, nav_processing):
        self.ros_communicator = ros_communicator
        self.nav_processing = nav_processing        

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
        else:
            pass
    def auto_control(self, mode="auto_nav", target=None, key=None):
        """
        自動控制邏輯
        Args:
            mode: 控制模式 ("auto_nav" 或 "manual_nav")
            target: 目標座標 (用於 manual_nav 模式)
            key: 鍵盤輸入
        """
        # 如果有按鍵輸入
        if key == "q":
            # 按下 q 時停止導航並退出
            action_key = self.nav_processing.stop_nav()
            self.ros_communicator.publish_car_control(action_key, publish_rear=True, publish_front=True)
        # 根據模式執行導航
        else:
            if mode == "auto_nav":
                action_key = self.nav_processing.get_action_from_nav2_plan(goal_coordinates=None)
            elif mode == "manual_nav":
                action_key = self.nav_processing.get_action_from_nav2_plan(goal_coordinates=target)
            self.ros_communicator.publish_car_control(action_key, publish_rear=True, publish_front=True)
