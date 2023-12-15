cmake_minimum_required(VERSION 3.5)
project(utilities_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(video_writer src/video_writer.cpp)
ament_target_dependencies(
  video_writer
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

add_executable(frame_saver src/frame_saver.cpp)
ament_target_dependencies(
  frame_saver
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV  
)

install(TARGETS
  video_writer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  frame_saver
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
