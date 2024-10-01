import orjson
from rclpy.node import Node
from std_msgs.msg import String
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl


class CarController(Node):
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

    def __init__(self, vel: float = 10):
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
        super().__init__("car_controller")
        self.vel = vel
        self._vel1 = 0  # rad/s
        self._vel2 = 0
        self._vel3 = 0
        self._vel4 = 0

        # ROS Publisher, initializes by publishing both rear and front control signals
        self.publisher_rear = self.create_publisher(String, DeviceDataTypeEnum.car_C_rear_wheel, 10)
        self.publisher_forward = self.create_publisher(String, DeviceDataTypeEnum.car_C_front_wheel, 10)
        self.publish_control(publish_rear=True, publish_front=True)

    def update_velocity(self, vel1, vel2, vel3, vel4):
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
        self._vel1 = vel1
        self._vel2 = vel2
        self._vel3 = vel3
        self._vel4 = vel4
        self.publish_control(publish_rear=True, publish_front=True)


    def publish_control(self, publish_rear=True, publish_front=True):
        """
        Publishes the control signals for the car's wheels. Can selectively publish rear or front wheels.

        Args:
            publish_rear (bool): Whether to publish the control signal for the rear wheels. Defaults to True.
            publish_front (bool): Whether to publish the control signal for the front wheels. Defaults to True.

        Example:
            car_controller.publish_control(publish_rear=True, publish_front=False)  # Only publish rear wheels control.
        """
        if publish_rear:
            control_signal_rear = {
                "type": str(DeviceDataTypeEnum.car_C_rear_wheel),
                "data": dict(CarCControl(target_vel=[self._vel1, self._vel2])),
            }
            control_msg_rear = String()
            control_msg_rear.data = orjson.dumps(control_signal_rear).decode()
            self.publisher_rear.publish(control_msg_rear)
            # self.get_logger().info(f"Published to rear: {control_msg_rear}")

        if publish_front:
            control_signal_front = {
                "type": str(DeviceDataTypeEnum.car_C_front_wheel),
                "data": dict(CarCControl(target_vel=[self._vel3, self._vel4])),
            }
            control_msg_front = String()
            control_msg_front.data = orjson.dumps(control_signal_front).decode()
            self.publisher_forward.publish(control_msg_front)
            # self.get_logger().info(f"Published to front: {control_msg_front}")

    def move_forward(self, vel):
        """
        Moves the car forward by updating all wheels to the same velocity.

        Args:
            vel (float): The velocity to set for all wheels (rad/s).

        Example:
            car_controller.move_forward(20)  # Move the car forward at 20 rad/s.
        """
        self.update_velocity(vel, vel, vel, vel)

    def move_backward(self, vel):
        """
        Moves the car backward by updating all wheels to the negative of the given velocity.

        Args:
            vel (float): The velocity to set for all wheels (rad/s), in negative for backward movement.

        Example:
            car_controller.move_backward(15)  # Move the car backward at 15 rad/s.
        """
        self.update_velocity(-vel, -vel, -vel, -vel)

    def turn_left(self, vel):
        """
        Turns the car left by reducing the velocity of the left wheels.

        Args:
            vel (float): The base velocity for the wheels, with the left wheels moving slower.

        Example:
            car_controller.turn_left(10)  # Turn the car left with a base velocity of 10 rad/s.
        """
        self.update_velocity(vel / 5, vel, vel / 5, vel)

    def turn_right(self, vel):
        """
        Turns the car right by reducing the velocity of the right wheels.

        Args:
            vel (float): The base velocity for the wheels, with the right wheels moving slower.

        Example:
            car_controller.turn_right(10)  # Turn the car right with a base velocity of 10 rad/s.
        """
        self.update_velocity(vel, vel / 5, vel, vel / 5)

    def rotate_cw(self, vel):
        """
        Rotates the car clockwise by setting opposite velocities for left and right wheels.

        Args:
            vel (float): The velocity for rotating the car, with left wheels moving backward and right wheels moving forward.

        Example:
            car_controller.rotate_cw(12)  # Rotate the car clockwise at a velocity of 12 rad/s.
        """
        self.update_velocity(-vel, vel, -vel, vel)

    def rotate_ccw(self, vel):
        """
        Rotates the car counterclockwise by setting opposite velocities for left and right wheels.

        Args:
            vel (float): The velocity for rotating the car, with left wheels moving forward and right wheels moving backward.

        Example:
            car_controller.rotate_ccw(12)  # Rotate the car counterclockwise at a velocity of 12 rad/s.
        """
        self.update_velocity(vel, -vel, vel, -vel)

    def stop(self):
        """
        Stops the car by setting the velocity of all wheels to zero.

        Example:
            car_controller.stop()  # Stop the car by setting all wheels' velocity to 0.
        """
        self.update_velocity(0, 0, 0, 0)
