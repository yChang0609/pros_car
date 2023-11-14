colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install 
source install/setup.bash
# ros2 run pros_arm_py keyboard
# ros2 run pros_arm_py random
ros2 run pros_car_py carA_random
# ros2 run pros_car_py carA_reader
# ros2 run pros_arm_py arm_reader
# ros2 run pros_arm random_target
# ros2 launch pros_arm_py launch_arm_reader_and_writer.py serial_port:=/dev/tty-esp32
