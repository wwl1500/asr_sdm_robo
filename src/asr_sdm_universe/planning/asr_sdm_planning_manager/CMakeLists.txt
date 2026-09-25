cmake_minimum_required(VERSION 3.8)
project(asr_sdm_planning_manager)

# Honor <PackageName>_ROOT / <PACKAGENAME>_ROOT variables so PCL/FLANN config
# modules stop emitting CMP0074 / CMP0144 developer warnings. The DEFAULT_*
# variables force the policy into the bundled PCL/FLANN scripts, which otherwise
# reset newer policies via cmake_policy(VERSION ...).
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_esdf_map REQUIRED)
find_package(asr_sdm_guidance_planner REQUIRED)
find_package(asr_sdm_local_path_modifier REQUIRED)
find_package(bspline REQUIRED)
find_package(asr_sdm_trajectory_generator REQUIRED)
find_package(asr_sdm_trajectory_optimizer REQUIRED)
find_package(asr_sdm_trajectory_visualizer REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
find_package(asr_sdm_log_collector REQUIRED)

# Generate the Bspline message
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Bspline.msg
  DEPENDENCIES builtin_interfaces geometry_msgs
)

set(asr_sdm_planning_manager_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  builtin_interfaces
  cv_bridge
  asr_sdm_control_msgs
  asr_sdm_guidance_planner
  asr_sdm_local_path_modifier
  bspline
  asr_sdm_trajectory_generator
  asr_sdm_trajectory_optimizer
  asr_sdm_trajectory_visualizer
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Main planner node
add_executable(planning_manager_node
  src/planning_manager_node.cpp
  src/topo_replan_fsm.cpp
)
ament_target_dependencies(planning_manager_node ${asr_sdm_planning_manager_dependencies})
target_link_libraries(planning_manager_node
  spdlog::spdlog
  asr_sdm_log_collector::asr_sdm_log_client
)

# Trajectory server
add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server ${asr_sdm_planning_manager_dependencies})
target_link_libraries(traj_server
  spdlog::spdlog
  asr_sdm_log_collector::asr_sdm_log_client
)

# NOTE:
# asr_sdm_esdf_map currently exports a broken imported target
# (asr_sdm_esdf_map::obj_generator). Using ament_target_dependencies() for it
# triggers a CMake error even though this package only needs headers from it.
if(DEFINED asr_sdm_esdf_map_INCLUDE_DIRS)
  target_include_directories(planning_manager_node PUBLIC ${asr_sdm_esdf_map_INCLUDE_DIRS})
  target_include_directories(traj_server PUBLIC ${asr_sdm_esdf_map_INCLUDE_DIRS})
endif()
if(TARGET asr_sdm_esdf_map::asr_sdm_esdf_map)
  target_link_libraries(planning_manager_node asr_sdm_esdf_map::asr_sdm_esdf_map)
  target_link_libraries(traj_server asr_sdm_esdf_map::asr_sdm_esdf_map)
endif()

# Link against the message interfaces generated in this package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(planning_manager_node "${cpp_typesupport_target}")
target_link_libraries(traj_server "${cpp_typesupport_target}")

target_link_libraries(planning_manager_node ${PCL_LIBRARIES})
target_link_libraries(traj_server ${PCL_LIBRARIES})

if(TARGET Eigen3::Eigen)
  target_link_libraries(planning_manager_node Eigen3::Eigen)
  target_link_libraries(traj_server Eigen3::Eigen)
endif()

install(TARGETS
  planning_manager_node
  traj_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
