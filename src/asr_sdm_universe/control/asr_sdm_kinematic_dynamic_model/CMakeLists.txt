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
  src/joint_dynamics_model.cpp
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

# GRAMPC MPC library (optional)
# Search in multiple possible locations for GRAMPC library and headers
set(GRAMPC_FOUND_TMP FALSE)

# Check for GRAMPC in various locations
set(GRAMPC_SEARCH_PREFIXES
  /home/wwlwwl/asr_sdm_robo/install/grampc  # ROS install
  /home/wwlwwl/grampc                       # Source build
  /home/wwlwwl/.local
  $ENV{HOME}/grampc
)

# Find GRAMPC library
find_library(GRAMPC_LIBRARY NAMES grampc PATHS ${GRAMPC_SEARCH_PREFIXES} NO_DEFAULT_PATH)
if(NOT GRAMPC_LIBRARY)
  find_library(GRAMPC_LIBRARY NAMES grampc)
endif()

# Find GRAMPC include directory - try multiple patterns
set(GRAMPC_INCLUDE_DIR "")
foreach(prefix IN LISTS GRAMPC_SEARCH_PREFIXES)
  # Pattern 1: prefix/include/grampc/grampc.h (ROS install)
  if(EXISTS "${prefix}/include/grampc/grampc.h")
    set(GRAMPC_INCLUDE_DIR "${prefix}/include/grampc")
    message(STATUS "GRAMPC include found at: ${GRAMPC_INCLUDE_DIR}")
    break()
  endif()
  # Pattern 2: prefix/include/grampc/... (nested grampc dir)
  if(EXISTS "${prefix}/include/grampc/grampc/grampc.h")
    set(GRAMPC_INCLUDE_DIR "${prefix}/include/grampc")
    message(STATUS "GRAMPC include found at: ${GRAMPC_INCLUDE_DIR}")
    break()
  endif()
  # Pattern 3: prefix/include/grampc.h (flat structure)
  if(EXISTS "${prefix}/include/grampc.h")
    set(GRAMPC_INCLUDE_DIR "${prefix}/include")
    message(STATUS "GRAMPC include found at: ${GRAMPC_INCLUDE_DIR}")
    break()
  endif()
  # Pattern 4: prefix/include/grampc-2.3/grampc.h
  if(EXISTS "${prefix}/include/grampc-2.3/grampc.h")
    set(GRAMPC_INCLUDE_DIR "${prefix}/include/grampc-2.3")
    message(STATUS "GRAMPC include found at: ${GRAMPC_INCLUDE_DIR}")
    break()
  endif()
endforeach()

# If not found, try standard paths
if(NOT GRAMPC_INCLUDE_DIR)
  find_path(GRAMPC_INCLUDE_DIR NAMES grampc.h)
endif()

# Find C++ header (problem_description.hpp)
set(GRAMPC_CPP_INCLUDE_DIR "")
foreach(prefix IN LISTS GRAMPC_SEARCH_PREFIXES)
  if(EXISTS "${prefix}/include/grampc/grampc/eigen/problem_description.hpp")
    set(GRAMPC_CPP_INCLUDE_DIR "${prefix}/include/grampc/grampc/eigen")
    break()
  endif()
  if(EXISTS "${prefix}/cpp/include/grampc.hpp")
    set(GRAMPC_CPP_INCLUDE_DIR "${prefix}/cpp/include")
    break()
  endif()
  if(EXISTS "${prefix}/.local/include/grampc-2.3/grampc.hpp")
    set(GRAMPC_CPP_INCLUDE_DIR "${prefix}/.local/include/grampc-2.3")
    break()
  endif()
endforeach()

# Find grampc_s library (C++ interface)
find_library(GRAMPC_S_LIBRARY NAMES grampc_s PATHS ${GRAMPC_SEARCH_PREFIXES} NO_DEFAULT_PATH)
if(NOT GRAMPC_S_LIBRARY)
  find_library(GRAMPC_S_LIBRARY NAMES grampc_s)
endif()

# Find grampc_s include directory (for grampc_s.hpp)
set(GRAMPC_S_INCLUDE_DIR "")
foreach(prefix IN LISTS GRAMPC_SEARCH_PREFIXES)
  if(EXISTS "${prefix}/include/grampc/grampc_s/grampc_s.hpp")
    set(GRAMPC_S_INCLUDE_DIR "${prefix}/include/grampc/grampc_s")
    break()
  endif()
endforeach()

if(GRAMPC_LIBRARY AND GRAMPC_INCLUDE_DIR)
  set(GRAMPC_FOUND_TMP TRUE)
  message(STATUS "Found GRAMPC: ${GRAMPC_LIBRARY}")
  message(STATUS "GRAMPC include: ${GRAMPC_INCLUDE_DIR}")
  if(GRAMPC_CPP_INCLUDE_DIR)
    message(STATUS "GRAMPC C++ include: ${GRAMPC_CPP_INCLUDE_DIR}")
  endif()
  if(GRAMPC_S_INCLUDE_DIR)
    message(STATUS "GRAMPC S include: ${GRAMPC_S_INCLUDE_DIR}")
  endif()
endif()

if(GRAMPC_FOUND_TMP)
  message(STATUS "Building MPC components with GRAMPC")
  
  # Collect all include directories
  set(GRAMPC_ALL_INCLUDES "${GRAMPC_INCLUDE_DIR}")
  if(GRAMPC_CPP_INCLUDE_DIR)
    list(APPEND GRAMPC_ALL_INCLUDES "${GRAMPC_CPP_INCLUDE_DIR}")
  endif()
  if(GRAMPC_S_INCLUDE_DIR)
    list(APPEND GRAMPC_ALL_INCLUDES "${GRAMPC_S_INCLUDE_DIR}")
  endif()
  
  # Create imported target for GRAMPC with library and include directories
  add_library(grampc::grampc INTERFACE IMPORTED)
  set_target_properties(grampc::grampc PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GRAMPC_ALL_INCLUDES}"
  )
  
  # Link both grampc (C library) and grampc_s (C++ library)
  if(WIN32)
    target_link_libraries(grampc::grampc INTERFACE ${GRAMPC_LIBRARY} ${GRAMPC_S_LIBRARY} winmm)
  else()
    target_link_libraries(grampc::grampc INTERFACE ${GRAMPC_LIBRARY} ${GRAMPC_S_LIBRARY} m)
  endif()
  
  add_library(asr_sdm_mpc_controller STATIC
    src/grampc_dynamics_interface.cpp
    src/underwater_mpc_controller.cpp
    src/mpc_safety_mapper.cpp
    src/stochastic_grampc_dynamics.cpp
  )
  set_target_properties(asr_sdm_mpc_controller PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )
  target_include_directories(asr_sdm_mpc_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(asr_sdm_mpc_controller PUBLIC
    asr_sdm_kinematic_dynamic_model_core
    grampc::grampc
    Eigen3::Eigen
  )
  
  install(TARGETS
    asr_sdm_mpc_controller
    EXPORT asr_sdm_kinematic_dynamic_model_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
else()
  message(STATUS "GRAMPC not found, skipping MPC components")
  message(STATUS "To enable MPC: Install GRAMPC from https://github.com/grampc/grampc")
  message(STATUS "GRAMPC_LIBRARY=${GRAMPC_LIBRARY}, GRAMPC_INCLUDE_DIR=${GRAMPC_INCLUDE_DIR}")
endif()

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
  FILES README.md
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
  find_package(asr_sdm_control_msgs REQUIRED)

  # add_executable(asr_sdm_kinematic_model_check
  #   test/asr_sdm_kinematic_model_check.cpp
  # )
  # target_link_libraries(asr_sdm_kinematic_model_check PUBLIC
  #   asr_sdm_kinematic_model
  #   ${asr_sdm_control_msgs_TARGETS}
  # )

  # add_test(
  #   NAME asr_sdm_kinematic_model_check
  #   COMMAND asr_sdm_kinematic_model_check
  # )

  # install(TARGETS asr_sdm_kinematic_model_check
  #   DESTINATION lib/${PROJECT_NAME}
  # )

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

  add_executable(underwater_dynamics_test test/underwater_dynamics_test.cpp)
  target_link_libraries(underwater_dynamics_test asr_sdm_kinematic_dynamic_model_core)
  target_compile_definitions(underwater_dynamics_test PRIVATE
    ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf")
  add_test(NAME underwater_dynamics_test COMMAND underwater_dynamics_test)

  add_executable(underwater_visualization_test test/underwater_visualization_test.cpp)
  target_link_libraries(underwater_visualization_test asr_sdm_kinematic_dynamic_model_core)
  add_test(NAME underwater_visualization_test COMMAND underwater_visualization_test)

  # GRAMPC MPC tests (only if GRAMPC is available)
  if(GRAMPC_FOUND_TMP)
    message(STATUS "Adding GRAMPC MPC tests")
    
    add_executable(grampc_interface_test test/grampc_interface_test.cpp)
    target_link_libraries(grampc_interface_test PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(grampc_interface_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME grampc_interface_test COMMAND grampc_interface_test)
    
    add_executable(dynamics_validation_test test/dynamics_validation_test.cpp)
    target_link_libraries(dynamics_validation_test PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(dynamics_validation_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME dynamics_validation_test COMMAND dynamics_validation_test)

    # Stochastic MPC (GRAMPC-S uncertainty propagation + chance constraints)
    add_executable(stochastic_mpc_test test/stochastic_mpc_test.cpp)
    target_link_libraries(stochastic_mpc_test PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(stochastic_mpc_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME stochastic_mpc_test COMMAND stochastic_mpc_test)

    # Stochastic MPC closed-loop validation.
    add_executable(stochastic_mpc_closed_loop_test
      test/stochastic_mpc_closed_loop_test.cpp)
    target_link_libraries(stochastic_mpc_closed_loop_test PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(stochastic_mpc_closed_loop_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME stochastic_mpc_closed_loop_test COMMAND stochastic_mpc_closed_loop_test)

    # MpcSafetyMapper unit tests.
    if(BUILD_TESTING)
      find_package(ament_cmake_gtest REQUIRED)
      ament_add_gtest(mpc_safety_mapper_test
        test/mpc_safety_mapper_test.cpp
      )
      target_link_libraries(mpc_safety_mapper_test
        asr_sdm_mpc_controller
        asr_sdm_kinematic_dynamic_model_core
      )
    endif()

    # Standalone reproducer / closed-loop demo for the GRAMPC run() regression.
    add_executable(grampc_run_repro examples/grampc_run_repro.cpp)
    target_link_libraries(grampc_run_repro PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(grampc_run_repro PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME grampc_run_repro COMMAND grampc_run_repro)

    # Multi-scenario MPC validation (10 scenarios).
    add_executable(mpc_scenario_test examples/mpc_scenario_test.cpp)
    target_link_libraries(mpc_scenario_test PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(mpc_scenario_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME mpc_scenario_test COMMAND mpc_scenario_test)

    # Time-varying trajectory MPC tracking example.
    add_executable(mpc_trajectory_tracking_example
      examples/mpc_trajectory_tracking_example.cpp)
    target_link_libraries(mpc_trajectory_tracking_example PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(mpc_trajectory_tracking_example PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME mpc_trajectory_tracking_example COMMAND mpc_trajectory_tracking_example)

    # MpcSafetyMapper closed-loop integration demo.
    add_executable(mpc_safety_mapper_example examples/mpc_safety_mapper_example.cpp)
    target_link_libraries(mpc_safety_mapper_example PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(mpc_safety_mapper_example PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME mpc_safety_mapper_example COMMAND mpc_safety_mapper_example)

    # Multi-operating-point closed-loop MPC validation across 6 regimes.
    add_executable(multi_operating_point_validation
      examples/multi_operating_point_validation.cpp)
    target_link_libraries(multi_operating_point_validation PRIVATE
      asr_sdm_mpc_controller
      asr_sdm_kinematic_dynamic_model_core
      grampc::grampc
    )
    target_compile_definitions(multi_operating_point_validation PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
    add_test(NAME multi_operating_point_validation COMMAND multi_operating_point_validation)

    # Thrust direction debug test
    add_executable(thrust_debug_test test/thrust_debug_test.cpp)
    target_link_libraries(thrust_debug_test PRIVATE
      asr_sdm_kinematic_dynamic_model_core
    )
    target_compile_definitions(thrust_debug_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )

    # Coordinate system fix verification test
    add_executable(coordinate_fix_test test/coordinate_fix_test.cpp)
    target_link_libraries(coordinate_fix_test PRIVATE
      asr_sdm_kinematic_dynamic_model_core
    )
    target_compile_definitions(coordinate_fix_test PRIVATE
      ASR_SDM_GENERATED_URDF="${CMAKE_BINARY_DIR}/../asr_sdm/generated/asr_sdm_segments_4.urdf"
    )
  endif()

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
