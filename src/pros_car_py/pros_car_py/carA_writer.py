
import json
from geometry_msgs.msg import Twist
from serial import Serial
from rclpy.node import Node

SERIAL_DEV_DEFAULT = '/dev/ttyUSB0'  # Replace with your default serial device

class CarAWriter(Node):
    def __init__(self):
        super().__init__('carA_writer')

        # Set up the serial connection
        serial_port = self.declare_parameter('serial_port', SERIAL_DEV_DEFAULT).value
        self._serial = Serial(serial_port, 115200, timeout=0)
        
        # Subscribe to /cmd_vel
        self._subscriber = self.create_subscription(
            Twist,               # Message type
            '/cmd_vel',          # Topic
            self.cmd_vel_callback,  # Callback function
            10                   # Queue size (optional)
        )

    def cmd_vel_callback(self, msg):
        """
        Callback for processing incoming /cmd_vel messages.

        :param msg: The Twist message received from the /cmd_vel topic.
        """
        # Convert the Twist message to JSON format 
        # unit of target_vel is rad per sec
        command_dict = {
            "target_vel": [msg.linear.x, msg.linear.x],  # Assuming the target velocity is the same for both wheels
            "direction": msg.angular.z * 90 / 0.5 + 90,  # Example conversion formula: convert angular speed to a range of 70-110
        }
        
        # Clip the direction value to be within the allowed range if necessary
        command_dict["direction"] = max(min(command_dict["direction"], 110), 70)

        # Serialize the command to a JSON string
        command_str = json.dumps(command_dict)
        
        # Write the JSON string to the serial device
        self._serial.write(command_str.encode('utf-8'))
        
        # Log the JSON command for debugging
        self.get_logger().info(f"Sent command: {command_str}")

# The rest of your node's code would go here


def main(args=None):
    rclpy.init(args=args)
    serial_writer = CarAWriter()
    rclpy.spin(serial_writer)

    serial_writer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
