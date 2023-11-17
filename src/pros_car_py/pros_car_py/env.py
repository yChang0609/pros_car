import os

SERIAL_DEV_DEFAULT = os.getenv('SERIAL_DEV_DEFAULT', '/dev/ttyUSB1')  # replace with your default value
SERIAL_DEV_FORWARD_DEFAULT = os.getenv('SERIAL_DEV_FORWARD_DEFAULT', '/dev/ttyUSB0')  # replace with your default value
SERIAL_BACK_DEFAULT = os.getenv('SERIAL_BACK_DEFAULT', '/dev/ttyUSB0')  # replace with your default value
SERIAL_FRONT_DEFAULT = os.getenv('SERIAL_FRONT_DEFAULT', '/dev/ttyUSB1')  # replace with your default value
