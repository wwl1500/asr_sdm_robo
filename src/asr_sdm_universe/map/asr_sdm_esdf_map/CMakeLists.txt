cmake_minimum_required(VERSION 3.8)
project(asr_sdm_esdf_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(asr_sdm_esdf_map SHARED
  src/esdf_map.cpp
  src/obj_predictor.cpp
  src/edt_environment.cpp
  src/raycast.cpp
)

target_include_directories(asr_sdm_esdf_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(asr_sdm_esdf_map
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(asr_sdm_esdf_map Eigen3::Eigen)
else()
  target_include_directories(asr_sdm_esdf_map PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(asr_sdm_esdf_map ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(obj_generator
  src/obj_generator.cpp
)

add_executable(esdf_map_node
  src/esdf_map_node.cpp
)

target_include_directories(obj_generator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(esdf_map_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(obj_generator
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(esdf_map_node
  rclcpp
)

target_link_libraries(esdf_map_node
  asr_sdm_esdf_map
)

install(TARGETS asr_sdm_esdf_map obj_generator esdf_map_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

ament_package()
