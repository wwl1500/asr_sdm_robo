cmake_minimum_required(VERSION 3.8)
project(asr_sdm_front_following_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_kinematic_dynamic_model REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(asr_sdm_front_following_control_core
  src/controller_visualization.cpp
  src/front_following_controller.cpp
)
target_include_directories(asr_sdm_front_following_control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(asr_sdm_front_following_control_core
  PUBLIC
    asr_sdm_kinematic_dynamic_model::underwater_dynamics
    pinocchio::pinocchio
    Eigen3::Eigen
    ${asr_sdm_control_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

add_executable(asr_sdm_front_following
  src/asr_sdm_front_following_node.cpp
)
target_link_libraries(asr_sdm_front_following
  asr_sdm_front_following_control_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
)

add_executable(controller_visualization_node
  src/controller_visualization_node.cpp
)
target_link_libraries(controller_visualization_node
  asr_sdm_front_following_control_core
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

install(
  TARGETS asr_sdm_front_following_control_core
  EXPORT asr_sdm_front_following_control_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    asr_sdm_front_following
    controller_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(asr_sdm_front_following_control_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  asr_sdm_control_msgs
  asr_sdm_kinematic_dynamic_model
  Eigen3
  geometry_msgs
  nav_msgs
  pinocchio
  sensor_msgs
  std_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Ported algorithm sources keep their original formatting.
  set(AMENT_LINT_AUTO_EXCLUDE "ament_cmake_uncrustify;ament_uncrustify;uncrustify")
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_front_following_controller
    test/test_front_following_controller.cpp
  )
  if(TARGET test_front_following_controller)
    target_link_libraries(test_front_following_controller
      asr_sdm_front_following_control_core
      ament_index_cpp::ament_index_cpp
      pinocchio::pinocchio
      Eigen3::Eigen
    )
  endif()

  ament_add_gtest(test_controller_visualization
    test/test_controller_visualization.cpp
  )
  if(TARGET test_controller_visualization)
    target_link_libraries(test_controller_visualization
      asr_sdm_front_following_control_core
      ${asr_sdm_control_msgs_TARGETS}
      Eigen3::Eigen
    )
  endif()
endif()

ament_package()
