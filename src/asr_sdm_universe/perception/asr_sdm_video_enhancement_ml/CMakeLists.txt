cmake_minimum_required(VERSION 3.8)
project(asr_sdm_video_enhancement_ml)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# If ONNXRUNTIME_ROOT was first configured with an empty env, it stays empty in the
# CMake cache unless we FORCE-update when the environment is set later (e.g. after
# adding ONNXRUNTIME_ROOT to ~/.bashrc).
if(NOT "$ENV{ONNXRUNTIME_ROOT}" STREQUAL "")
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "Path to ONNX Runtime root" FORCE)
else()
  set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_ROOT}" CACHE PATH "Path to ONNX Runtime root")
endif()
if(NOT ONNXRUNTIME_ROOT)
  message(FATAL_ERROR
    "ONNXRUNTIME_ROOT is not set. Please set the ONNXRUNTIME_ROOT environment variable "
    "or pass -DONNXRUNTIME_ROOT=/path/to/onnxruntime to CMake.")
endif()

set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime headers not found at ${ONNXRUNTIME_INCLUDE_DIR}. Set ONNXRUNTIME_ROOT.")
endif()
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
  message(FATAL_ERROR "ONNX Runtime library not found at ${ONNXRUNTIME_LIBRARY}. Set ONNXRUNTIME_ROOT.")
endif()

add_executable(asr_sdm_video_enhancement_ml_node src/asr_sdm_video_enhancement_ml_node.cpp)
target_include_directories(asr_sdm_video_enhancement_ml_node PRIVATE
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(asr_sdm_video_enhancement_ml_node
  ${ONNXRUNTIME_LIBRARY}
  ${OpenCV_LIBS}
)
ament_target_dependencies(asr_sdm_video_enhancement_ml_node
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)
set_target_properties(asr_sdm_video_enhancement_ml_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

install(TARGETS asr_sdm_video_enhancement_ml_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
