cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch
  maps
  worlds
  robot
  config
  
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  nodes/patrol_manager.py  # <--- Changed file name
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
