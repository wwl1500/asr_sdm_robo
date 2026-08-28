cmake_minimum_required(VERSION 3.8)
project(asr_sdm_control_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ===================================================
# Message & Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UnitCmd.msg"
  "msg/UnitStatus.msg"
  "msg/ActuatorCmd.msg"
  "msg/ActuatorForceCmd.msg"
  "msg/RobotStatus.msg"
  "msg/RobotState.msg"
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/OutputData.msg"
  "msg/RobotCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/LQRTrajectory.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

add_library(decode_msgs SHARED src/decode_msgs.cpp)
add_library(encode_msgs SHARED src/encode_msgs.cpp)

target_include_directories(decode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(encode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(decode_msgs geometry_msgs nav_msgs std_msgs)
ament_target_dependencies(encode_msgs geometry_msgs nav_msgs std_msgs)

if(TARGET Eigen3::Eigen)
  target_link_libraries(decode_msgs Eigen3::Eigen)
else()
  target_include_directories(decode_msgs PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(encode_msgs "${cpp_typesupport_target}")
target_link_libraries(decode_msgs "${cpp_typesupport_target}")

install(TARGETS decode_msgs encode_msgs
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs std_msgs Eigen3)
ament_package()
