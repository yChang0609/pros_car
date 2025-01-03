cmake_minimum_required(VERSION 3.8)
project(udp_exr_gpu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(nlohmann_json REQUIRED)

# 添加包含目錄
include_directories(
  include
  src
)

# 建立可執行檔
add_executable(udp_depth_exr_gpu_node src/udp_depth_exr_gpu_node.cpp)

# 連結必要的庫
target_link_libraries(udp_depth_exr_gpu_node
  OpenEXR::IlmImf  # 根據實際情況，可能是 OpenEXR::IlmImf 或其他
  nlohmann_json::nlohmann_json
)

# 設定 ROS 2 相依
ament_target_dependencies(udp_depth_exr_gpu_node
  rclcpp
  sensor_msgs
)

# 安裝可執行檔
install(TARGETS
  udp_depth_exr_gpu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
