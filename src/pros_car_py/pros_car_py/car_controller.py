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
        """
        Initializes the CarController node with default velocity and sets initial velocities for all wheels.

        Args:
            vel (float): Default velocity for the car's wheels. Defaults to 10.

        Attributes:
            vel (float): Default velocity for the car's wheels, assigned from the input parameter.
            _vel1 (float): Velocity for the rear left wheel, initialized to 0.
            _vel2 (float): Velocity for the rear right wheel, initialized to 0.
            _vel3 (float): Velocity for the front left wheel, initialized to 0.
            _vel4 (float): Velocity for the front right wheel, initialized to 0.

        Example:
            car_controller = CarController(vel=20)
            car_controller.move_forward(20)  # Move the car forward with a velocity of 20 rad/s.
        """
        vel = 0
        self.vel = vel
        self._vel1 = 0  # rad/s
        self._vel2 = 0
        self._vel3 = 0
        self._vel4 = 0
        self.ros_communicator = ros_communicator
        self.nav_processing = nav_processing

        # ROS Publisher, initializes by publishing both rear and front control signals
        
        
        # self.publish_control(publish_rear=True, publish_front=True)

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
        # self.publish_control(publish_rear=True, publish_front=True)


    # def publish_control(self, publish_rear=True, publish_front=True):
    #     """
    #     Publishes the control signals for the car's wheels. Can selectively publish rear or front wheels.

    #     Args:
    #         publish_rear (bool): Whether to publish the control signal for the rear wheels. Defaults to True.
    #         publish_front (bool): Whether to publish the control signal for the front wheels. Defaults to True.

    #     Example:
    #         car_controller.publish_control(publish_rear=True, publish_front=False)  # Only publish rear wheels control.
    #     """
    #     if publish_rear:
    #         control_signal_rear = {
    #             "type": str(DeviceDataTypeEnum.car_C_rear_wheel),
    #             "data": dict(CarCControl(target_vel=[self._vel1, self._vel2])),
    #         }
    #         control_msg_rear = String()
    #         control_msg_rear.data = orjson.dumps(control_signal_rear).decode()
    #         self.publisher_rear.publish(control_msg_rear)
    #         # self.get_logger().info(f"Published to rear: {control_msg_rear}")

    #     if publish_front:
    #         control_signal_front = {
    #             "type": str(DeviceDataTypeEnum.car_C_front_wheel),
    #             "data": dict(CarCControl(target_vel=[self._vel3, self._vel4])),
    #         }
    #         control_msg_front = String()
    #         control_msg_front.data = orjson.dumps(control_signal_front).decode()
    #         self.publisher_forward.publish(control_msg_front)
    #         # self.get_logger().info(f"Published to front: {control_msg_front}")


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
            self.get_logger().info(f"Invalid key: {key}")
    
    def auto_control(self, mode, target = None):
        if mode == "auto_nav":
            action_key = self.nav_processing.get_action()
            print("action_key", action_key)
            self.ros_communicator.publish_car_control(action_key, publish_rear=True, publish_front=True)
        elif mode == "manual_nav":
            pass
        pass
