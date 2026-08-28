cmake_minimum_required(VERSION 3.20)
project(asr_sdm_kinematic_dynamic_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(asr_sdm_kinematic_model STATIC
  src/asr_sdm_kinematic_model.cpp
)
set_target_properties(asr_sdm_kinematic_model PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(asr_sdm_kinematic_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(asr_sdm_kinematic_model PUBLIC
  pinocchio::pinocchio
  Eigen3::Eigen
)

add_library(asr_sdm_kinematic_dynamic_model_core STATIC
  src/math_types.cpp
  src/robot_model_3d.cpp
  src/jacobian_3d.cpp
  src/screw_drive_model.cpp
  src/fluid_force_model.cpp
  src/pinocchio_model.cpp
  src/actuator_model.cpp
  src/underwater_simulator.cpp
  src/underwater_visualization.cpp
  src/csv_logger.cpp
)
set_target_properties(asr_sdm_kinematic_dynamic_model_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
ament_target_dependencies(asr_sdm_kinematic_dynamic_model_core
  builtin_interfaces
  geometry_msgs
  visualization_msgs
)
target_include_directories(asr_sdm_kinematic_dynamic_model_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(asr_sdm_kinematic_dynamic_model_core
  pinocchio::pinocchio
  Eigen3::Eigen
)

add_executable(pinocchio_dynamics_node
  src/pinocchio_dynamics_node.cpp
)

target_include_directories(pinocchio_dynamics_node
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(pinocchio_dynamics_node
  PRIVATE
    ament_index_cpp::ament_index_cpp
    # pinocchio_headers adds include/pinocchio/deprecated (needed for some v3-era paths).
    pinocchio::pinocchio_headers
    pinocchio::pinocchio
    rclcpp::rclcpp
    Eigen3::Eigen
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

add_executable(underwater_simulator_node
  src/underwater_simulator_node.cpp
)
ament_target_dependencies(underwater_simulator_node
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(underwater_simulator_node
  asr_sdm_kinematic_dynamic_model_core
)

install(TARGETS
  asr_sdm_kinematic_model
  asr_sdm_kinematic_dynamic_model_core
  EXPORT asr_sdm_kinematic_dynamic_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS pinocchio_dynamics_node underwater_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz visualization
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS visualization/underwater_control_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README_simulator.md README_visualization.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(asr_sdm_kinematic_dynamic_model_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  builtin_interfaces
  geometry_msgs
  pinocchio
  visualization_msgs
)
ament_export_include_directories(include)

# Keep the legacy URDF dynamics utility sources unchanged while excluding them
# from the current model package lint set.
set(AMENT_LINT_AUTO_FILE_EXCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pinocchio_dynamics_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/pinocchio_dynamics.launch.py
)
set(AMENT_LINT_AUTO_EXCLUDE "ament_cmake_pep257")

if(BUILD_TESTING)
  find_package(asr_sdm_head_following_control REQUIRED)
  find_package(asr_sdm_control_msgs REQUIRED)

  add_executable(asr_sdm_kinematic_model_check
    test/asr_sdm_kinematic_model_check.cpp
  )
  target_link_libraries(asr_sdm_kinematic_model_check PUBLIC
    asr_sdm_kinematic_model
    asr_sdm_head_following_control::front_unit_following_controller_3d_core
    ${asr_sdm_control_msgs_TARGETS}
  )

  add_test(
    NAME asr_sdm_kinematic_model_check
    COMMAND asr_sdm_kinematic_model_check
  )

  install(TARGETS asr_sdm_kinematic_model_check
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(fluid_force_test test/fluid_force_test.cpp)
  target_link_libraries(fluid_force_test asr_sdm_kinematic_dynamic_model_core)
  add_test(NAME fluid_force_test COMMAND fluid_force_test)

  add_executable(model_3d_test test/model_3d_test.cpp)
  target_link_libraries(model_3d_test asr_sdm_kinematic_dynamic_model_core)
  target_compile_definitions(model_3d_test PRIVATE
    ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
  )
  add_test(NAME model_3d_test COMMAND model_3d_test)

  add_executable(underwater_simulator_test test/underwater_simulator_test.cpp)
  target_link_libraries(underwater_simulator_test asr_sdm_kinematic_dynamic_model_core)
  target_compile_definitions(underwater_simulator_test PRIVATE
    ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
  )
  add_test(NAME underwater_simulator_test COMMAND underwater_simulator_test)

  add_executable(underwater_visualization_test test/underwater_visualization_test.cpp)
  target_link_libraries(underwater_visualization_test asr_sdm_kinematic_dynamic_model_core)
  add_test(NAME underwater_visualization_test COMMAND underwater_visualization_test)

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

ament_package()
