cmake_minimum_required(VERSION 3.8)
project(track_car)

# 编译警告设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)


# 包含头文件目录
include_directories(include)

# ---------- 可执行节点 1：track_car_node ----------
add_executable(track_car_node
  src/track_car_node.cpp
)
ament_target_dependencies(track_car_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
)
install(TARGETS track_car_node DESTINATION lib/${PROJECT_NAME})

# ---------- 可执行节点 2：control_node ----------
add_executable(control_node
  src/control_node.cpp
)
ament_target_dependencies(control_node
  rclcpp
  geometry_msgs
)
install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})

# ---------- 可执行节点 3：memory_drive_node ----------
# add_executable(memory_drive_node
#   src/MemoryDrive.cpp
#   src/MemoryTrajectoryGenerate.cpp
#   src/AutoDriveControl.cpp
# )
# ament_target_dependencies(memory_drive_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   tf2
# )
# install(TARGETS memory_drive_node DESTINATION lib/${PROJECT_NAME})

# ---------- 可执行节点 4：alg_main（主程序入口） ----------
add_executable(alg_main
  src/alg_main.cpp
  src/MemoryDrive.cpp 
  src/MemoryTrajectoryGenerate.cpp
  src/AutoDriveControl.cpp
)
target_link_libraries(alg_main
  Geographic
)
ament_target_dependencies(alg_main
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
)




install(TARGETS alg_main DESTINATION lib/${PROJECT_NAME})

# 结束
ament_package()
