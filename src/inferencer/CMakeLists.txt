cmake_minimum_required(VERSION 3.8)
project(inferencer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV REQUIRED)

include_directories(include)

# The backends are plugins: nn_controller loads lib<backend>_inferencer.so at runtime with boost::dll.
# They must be SHARED libraries (catkin builds shared libraries by default, ament does not), and they keep
# the RPATH of their link directories after install so that dlopen can find CUDA/TensorFlow libraries.
set(INFERENCER_BACKENDS "")

# -------------------------------
# TensorRT backend
find_package(CUDA REQUIRED)

find_library(NVINFER_LIBRARY nvinfer PATHS /usr/lib/aarch64-linux-gnu/)
find_library(NVINFER_PLUGIN_LIBRARY nvinfer_plugin PATHS /usr/lib/aarch64-linux-gnu/)
find_library(NVONNXPARSER_LIBRARY nvonnxparser PATHS /usr/lib/aarch64-linux-gnu/)

if(NOT NVINFER_LIBRARY OR NOT NVINFER_PLUGIN_LIBRARY OR NOT NVONNXPARSER_LIBRARY)
  message(FATAL_ERROR "TensorRT libraries (nvinfer, nvinfer_plugin, nvonnxparser) not found")
endif()

set(TENSORRT_LIBRARIES
  ${NVINFER_LIBRARY}
  ${NVINFER_PLUGIN_LIBRARY}
  ${NVONNXPARSER_LIBRARY}
)

add_library(tensorrt_inferencer SHARED src/tensorrt_inferencer.cpp)
target_include_directories(tensorrt_inferencer PRIVATE ${CUDA_INCLUDE_DIRS})
ament_target_dependencies(tensorrt_inferencer rclcpp)
target_link_libraries(tensorrt_inferencer
  ${CUDA_LIBRARIES}
  ${TENSORRT_LIBRARIES}
  Boost::filesystem
)
set_target_properties(tensorrt_inferencer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND INFERENCER_BACKENDS tensorrt_inferencer)

# -------------------------------
# TensorFlow backend (optional, uses the C++ libraries shipped with the TensorFlow pip package)
find_program(Python3_EXECUTABLE NAMES python3)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import tensorflow as tf; print('TF_CONFIG=' + '|'.join([tf.sysconfig.get_include(), tf.sysconfig.get_lib()] + [f[2:] for f in tf.sysconfig.get_compile_flags() if f.startswith('-D')]))"
  RESULT_VARIABLE TF_RESULT
  OUTPUT_VARIABLE TF_OUTPUT
  ERROR_QUIET
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "TF_CONFIG=[^\n]*" TF_CONFIG "${TF_OUTPUT}")

if(TF_RESULT EQUAL 0 AND TF_CONFIG)
  string(REPLACE "TF_CONFIG=" "" TF_CONFIG "${TF_CONFIG}")
  string(REPLACE "|" ";" TF_CONFIG "${TF_CONFIG}")
  list(GET TF_CONFIG 0 TENSORFLOW_INCLUDE_DIR)
  list(GET TF_CONFIG 1 TENSORFLOW_LIBRARY_DIR)
  list(SUBLIST TF_CONFIG 2 -1 TENSORFLOW_DEFINITIONS)

  # TF_Version is defined in libtensorflow_cc, LoadSavedModel/Tensor in libtensorflow_framework
  set(TENSORFLOW_LIBRARIES
    ${TENSORFLOW_LIBRARY_DIR}/libtensorflow_framework.so.2
    ${TENSORFLOW_LIBRARY_DIR}/libtensorflow_cc.so.2
  )
endif()

if(TENSORFLOW_LIBRARIES AND EXISTS ${TENSORFLOW_LIBRARY_DIR}/libtensorflow_framework.so.2
   AND EXISTS ${TENSORFLOW_LIBRARY_DIR}/libtensorflow_cc.so.2)
  message(STATUS "TensorFlow include directory: ${TENSORFLOW_INCLUDE_DIR}")
  message(STATUS "TensorFlow library directory: ${TENSORFLOW_LIBRARY_DIR}")
  message(STATUS "TensorFlow definitions: ${TENSORFLOW_DEFINITIONS}")

  add_library(tensorflow_inferencer SHARED src/tensorflow_inferencer.cpp)
  target_include_directories(tensorflow_inferencer PRIVATE ${TENSORFLOW_INCLUDE_DIR})
  target_compile_definitions(tensorflow_inferencer PRIVATE ${TENSORFLOW_DEFINITIONS})
  target_link_libraries(tensorflow_inferencer
    ${TENSORFLOW_LIBRARIES}
    Boost::filesystem
  )
  set_target_properties(tensorflow_inferencer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  list(APPEND INFERENCER_BACKENDS tensorflow_inferencer)
else()
  message(WARNING "TensorFlow C++ libraries not found, tensorflow_inferencer will not be built")
endif()

# -------------------------------
# Command line tool to run a backend without ROS (used to verify the backends)
add_executable(inferencer_cli tools/inferencer_cli.cpp)
target_include_directories(inferencer_cli PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(inferencer_cli
  ${OpenCV_LIBS}
  Boost::filesystem
  ${CMAKE_DL_LIBS}
)

# -------------------------------
# Install
install(TARGETS ${INFERENCER_BACKENDS}
  LIBRARY DESTINATION lib
)

install(TARGETS inferencer_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
