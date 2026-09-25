cmake_minimum_required(VERSION 3.8)
project(asr_sdm_guidance_planner_dev)

# Keep third-party CMake package discovery quiet on Ubuntu 24.04 / ROS 2 Jazzy.
# PCL may call FindFLANN through exported dependencies; setting the policy defaults
# here prevents CMP0074/CMP0144 developer warnings without changing planner logic.
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

unset(FLANN_ROOT CACHE)
unset(FLANN_ROOT)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress CMake developer warnings from third-party packages" FORCE)



if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_GUIDANCE_PLANNER_TEST_NODE "Build RViz test node for guidance planner" ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(asr_sdm_lbfgs_solver REQUIRED)
find_package(asr_sdm_esdf_map REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/voxel_esdf_map.cpp
  src/astar_3d_planner.cpp
  src/sphere_corridor.cpp
  src/lbfgs_path_optimizer.cpp
  src/guidance_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)


if(DEFINED asr_sdm_esdf_map_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${asr_sdm_esdf_map_INCLUDE_DIRS})
endif()

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

if(TARGET asr_sdm_lbfgs_solver::asr_sdm_lbfgs_solver)
  target_link_libraries(${PROJECT_NAME} asr_sdm_lbfgs_solver::asr_sdm_lbfgs_solver)
else()
  target_link_libraries(${PROJECT_NAME} asr_sdm_lbfgs_solver)
endif()

ament_target_dependencies(${PROJECT_NAME} rclcpp asr_sdm_lbfgs_solver)

if(BUILD_GUIDANCE_PLANNER_TEST_NODE)
  find_package(ament_index_cpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)

  set(TEST_DEPENDENCIES
    rclcpp
    ament_index_cpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    pcl_conversions
  )

  add_executable(rviz_astar_lbfgs_planner
    test/guidance_planner_test.cpp
    test/rviz_planning_node.cpp
  )

  target_include_directories(rviz_astar_lbfgs_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  target_link_libraries(rviz_astar_lbfgs_planner ${PROJECT_NAME})
  ament_target_dependencies(rviz_astar_lbfgs_planner ${TEST_DEPENDENCIES})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_GUIDANCE_PLANNER_TEST_NODE)
  install(
    TARGETS rviz_astar_lbfgs_planner
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(
    DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
  install(
    DIRECTORY maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 PCL rclcpp asr_sdm_lbfgs_solver asr_sdm_esdf_map)
ament_package()
