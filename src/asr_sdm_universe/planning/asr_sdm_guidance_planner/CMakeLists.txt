cmake_minimum_required(VERSION 3.8)
project(asr_sdm_guidance_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(asr_sdm_esdf_map REQUIRED)
find_package(dubins_path_3d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/astar.cpp
  src/guidance_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(DEFINED asr_sdm_esdf_map_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${asr_sdm_esdf_map_INCLUDE_DIRS})
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# NOTE:
# asr_sdm_esdf_map exports a broken imported target (asr_sdm_esdf_map::obj_generator),
# so it cannot be used via ament_target_dependencies(). The legacy Astar sources pull in
# rclcpp and message headers transitively through the esdf_map env headers, so those
# dependencies are declared explicitly to get their include paths.
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  visualization_msgs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  Boost::boost
  dubins_path_3d::dubins_path_3d
  ${PCL_LIBRARIES}
)

# The legacy Astar sources call into EDTEnvironment / SDFMap, which live in the
# asr_sdm_esdf_map library. Link it directly (its ament target export is broken, but the
# library imported target itself is usable).
if(TARGET asr_sdm_esdf_map::asr_sdm_esdf_map)
  target_link_libraries(${PROJECT_NAME} asr_sdm_esdf_map::asr_sdm_esdf_map)
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
  Eigen3
  Boost
  PCL
  asr_sdm_esdf_map
  dubins_path_3d
  rclcpp
  geometry_msgs
  visualization_msgs
)
ament_package()
