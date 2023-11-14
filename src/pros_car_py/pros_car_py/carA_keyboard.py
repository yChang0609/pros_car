
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

class CarAKeyboardController(Node):
    def __init__(self, stdscr):
        super().__init__('car_A_keyboard')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            DeviceDataTypeEnum.car_A_state,
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            DeviceDataTypeEnum.car_A_control,# topic name
            10
        )
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, 'joint_trajectory_point', 10)
        self.joint_pos = [1.57, 1.57, 1.57, 1.57, 1.0]
        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        self._car_state_msg =""
        self._direction =90 # degree
        self._vel = 0 # rad/s

    def _sub_callback(self, msg):
        # Process the incoming message (if needed)
        # TODO show data in screen
        self._car_state_msg =  str(self.get_clock().now())+" "+msg.data
        
    def _pub_control(self):
        # Generate a random control signal
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_A_control),
            "data":dict(CarAControl(
                    direction=self._direction,
                    target_vel= [self._vel,self._vel]
                ))
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)

        self.get_logger().info(f'publish {control_msg}')

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
                    elif c == ord('z'):
                        self.handle_key_z()
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
                    elif c == ord('q'):  # Exit on 'q'
                        self._direction =90 # degree
                        self._vel = 0 # rad/s
                        self._pub_control()

                        break
                    self._pub_control()
                    self.pub_arm()
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
        self.stdscr.move(4, 0)
        self.stdscr.addstr(f"Arm pos : {self.joint_pos}")
        self.stdscr.move(5, 0)

        # self.get_logger().debug(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

    def handle_key_w(self):
        # Your action for the 'w' key here
        self.stdscr.addstr(f"car go forward")
        
        self._vel = 10 # rad/s
        # self.stdscr.move(1, 0)
        pass
    
    def handle_key_a(self):
        # Your action for the 'a' key here
        self.stdscr.addstr(f"car turn left ")
        self._direction =75 # degree
        
        pass

    # Add methods for other keys similarly
    def handle_key_s(self):
        self.stdscr.addstr(f"car go backward")
        self._vel = -10 # rad/s
        pass

    def handle_key_d(self):
        self.stdscr.addstr(f"car turn right")
        self._direction =105 # degree
        pass
    def handle_key_z(self):
        self.stdscr.addstr(f"stop car")
        self._direction =90 # degree
        self._vel = 0 # rad/s
        pass

    def handle_key_i(self):
        self.stdscr.addstr(f"arm rift up")
        self.joint_pos[2]+=0.05
        
        pass

    def handle_key_j(self):
        self.stdscr.addstr(f"arm turn left")
        self.joint_pos[0]-=0.05

        pass

    def handle_key_k(self):
        self.stdscr.addstr(f"arm rift down")
        self.joint_pos[2]-=0.05

        pass

    def handle_key_l(self):
        self.stdscr.addstr(f"arm turn right")
        self.joint_pos[0]+=0.05

        pass

    def handle_key_u(self):
        self.stdscr.addstr(f"arm j4 rotate left")
        self.joint_pos[3]-=0.05

        pass

    def handle_key_o(self):
        self.stdscr.addstr(f"arm j4 rotate right")
        self.joint_pos[3]+=0.05
        pass

    def handle_key_y(self):
        self.stdscr.addstr(f"arm catch!")
        self.joint_pos[4]=1.57
        pass

    def handle_key_h(self):
        self.stdscr.addstr(f"arm release!")
        self.joint_pos[4]=1.0


        pass
    def pub_arm(self):
        msg = JointTrajectoryPoint()
        msg.positions = self.joint_pos  # Replace with actual desired positions
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0] # Replace with actual desired velocities
        # You can set other fields of the JointTrajectoryPoint message similarly.
        self.joint_trajectory_publisher_.publish(msg)
# ... Rest of your code, e.g. initializing rclpy and running the node

def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = CarAKeyboardController(stdscr)

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
