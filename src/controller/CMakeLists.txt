

cmake_minimum_required(VERSION 3.8)
project(controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teacar_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(inferencer REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(cv_bridge REQUIRED)


# Include directories
include_directories(
  include
)

# -------------------------------
# Joystick controller executable
add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller rclcpp sensor_msgs std_msgs teacar_msgs)

# Include headers
target_include_directories(joystick_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# -------------------------------
# Param controller executable
add_executable(param_controller src/param_controller.cpp)
ament_target_dependencies(param_controller
  rclcpp std_msgs teacar_msgs)

target_include_directories(param_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# -------------------------------
# Neural network controller executable
# The inferencer backends are loaded at runtime with boost::dll, so they are not linked here
# Build the model-loading implementation; nn_controller.cpp is a camera-only diagnostic node.

# Experimental ROS2 NN controller with TensorRT inference

add_executable(new_nn_controller2 src/new_nn_controller2.cpp)

ament_target_dependencies(new_nn_controller2
  rclcpp
  sensor_msgs
  std_msgs
  teacar_msgs
  ament_index_cpp
  inferencer
  cv_bridge
)

target_link_libraries(new_nn_controller2
  ${OpenCV_LIBS}
  Boost::filesystem
  ${CMAKE_DL_LIBS}
)

target_include_directories(new_nn_controller2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)




ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(interfaces_msg)

install(TARGETS
  joystick_controller
  param_controller
  new_nn_controller2
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_package()


