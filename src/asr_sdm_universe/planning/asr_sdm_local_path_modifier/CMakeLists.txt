cmake_minimum_required(VERSION 3.8)
project(asr_sdm_local_path_modifier)

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

option(BUILD_LOCAL_PATH_MODIFIER_TEST_NODE "Build RViz test node for local path modifier" ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(asr_sdm_esdf_map REQUIRED)
find_package(asr_sdm_log_collector REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/topo_path_modifier.cpp
  src/topo_prm.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


if(DEFINED asr_sdm_esdf_map_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${asr_sdm_esdf_map_INCLUDE_DIRS})
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

# NOTE:
# asr_sdm_esdf_map exports a broken imported target (asr_sdm_esdf_map::obj_generator),
# so it cannot be used via ament_target_dependencies(). The legacy TopologyPRM sources
# pull in rclcpp and message headers transitively through the esdf_map env headers, so
# those dependencies are declared explicitly to get their include paths.
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  visualization_msgs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  asr_sdm_log_collector::asr_sdm_log_client
)

# The legacy TopologyPRM sources call into EDTEnvironment / SDFMap / RayCaster, which live
# in the asr_sdm_esdf_map library. Link it directly (its ament target export is broken, but
# the library imported target itself is usable).
if(TARGET asr_sdm_esdf_map::asr_sdm_esdf_map)
  target_link_libraries(${PROJECT_NAME} asr_sdm_esdf_map::asr_sdm_esdf_map)
endif()


if(BUILD_LOCAL_PATH_MODIFIER_TEST_NODE)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  set(TEST_DEPENDENCIES
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    visualization_msgs
  )

  add_executable(local_path_modifier_test test/local_path_modifier_test.cpp)
  target_link_libraries(local_path_modifier_test ${PROJECT_NAME})
  ament_target_dependencies(local_path_modifier_test ${TEST_DEPENDENCIES})
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

if(BUILD_LOCAL_PATH_MODIFIER_TEST_NODE)
  install(
    TARGETS local_path_modifier_test
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
endif()

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3 asr_sdm_esdf_map asr_sdm_log_collector rclcpp geometry_msgs visualization_msgs)
ament_package()
