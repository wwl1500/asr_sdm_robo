cmake_minimum_required(VERSION 3.20)
project(asr_sdm_kinematic_dynamic_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdfdom_headers REQUIRED)

add_library(underwater_dynamics
  src/underwater_dynamics.cpp
)

target_include_directories(underwater_dynamics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(underwater_dynamics
  PUBLIC
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
    underwater_dynamics
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

install(
  TARGETS underwater_dynamics
  DESTINATION lib
)

install(
  TARGETS pinocchio_dynamics_node
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_underwater_dynamics
    test/test_underwater_dynamics.cpp
  )

  if(TARGET test_underwater_dynamics)
    target_link_libraries(test_underwater_dynamics
      underwater_dynamics
      ament_index_cpp::ament_index_cpp
      pinocchio::pinocchio
      Eigen3::Eigen
    )
  endif()
endif()

ament_package()
