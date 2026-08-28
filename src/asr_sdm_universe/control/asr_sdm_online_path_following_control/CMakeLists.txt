cmake_minimum_required(VERSION 3.8)
project(asr_sdm_online_path_following_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(asr_sdm_head_following_control REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(online_path_following_controller_3d_core STATIC
  src/online_path_following_controller_3d.cpp
)
set_target_properties(online_path_following_controller_3d_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(online_path_following_controller_3d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(online_path_following_controller_3d_core PUBLIC
  asr_sdm_head_following_control::front_unit_following_controller_3d_core
)

add_executable(realtime_online_path_following_controller_3d
  src/realtime_online_path_following_controller_3d_node.cpp
)
target_link_libraries(realtime_online_path_following_controller_3d PUBLIC
  online_path_following_controller_3d_core
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

option(BUILD_ONLINE_PATH_FOLLOWING_PLOTS
  "Build the optional Matplot++ closed-loop validation artifact generator" OFF)

if(BUILD_ONLINE_PATH_FOLLOWING_PLOTS)
  find_package(Matplot++ CONFIG QUIET)
  find_package(Threads REQUIRED)
  if(NOT TARGET Matplot++::matplot AND DEFINED MATPLOTPP_SOURCE_DIR)
    add_subdirectory(
      "${MATPLOTPP_SOURCE_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/matplotplusplus"
      EXCLUDE_FROM_ALL
    )
  endif()

  if(NOT TARGET Matplot++::matplot)
    message(FATAL_ERROR
      "Matplot++ is required for BUILD_ONLINE_PATH_FOLLOWING_PLOTS. "
      "Install it with ./install_matplotplusplus.sh or set "
      "-DMATPLOTPP_SOURCE_DIR=/path/to/matplotplusplus.")
  endif()

  add_executable(online_path_following_controller_closed_loop_plot
    src/online_path_following_controller_closed_loop_plot.cpp
  )
  target_include_directories(online_path_following_controller_closed_loop_plot
    PRIVATE
      src
  )
  target_link_libraries(online_path_following_controller_closed_loop_plot
    online_path_following_controller_3d_core
    Matplot++::matplot
    Threads::Threads
  )

  install(
    TARGETS online_path_following_controller_closed_loop_plot
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS online_path_following_controller_3d_core
  EXPORT asr_sdm_online_path_following_control_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS realtime_online_path_following_controller_3d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(asr_sdm_online_path_following_control_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(asr_sdm_head_following_control)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(history_length_assertion_test
    test/history_length_assertion_test.cpp
  )
  if(TARGET history_length_assertion_test)
    target_link_libraries(history_length_assertion_test
      online_path_following_controller_3d_core
    )
  endif()

  ament_add_gtest(online_path_following_controller_chapter8_test
    test/online_path_following_controller_chapter8_test.cpp
  )
  if(TARGET online_path_following_controller_chapter8_test)
    target_include_directories(online_path_following_controller_chapter8_test
      PRIVATE
        src
    )
    target_link_libraries(online_path_following_controller_chapter8_test
      online_path_following_controller_3d_core
    )
  endif()

  ament_add_gtest(online_path_following_controller_closed_loop_test
    test/online_path_following_controller_closed_loop_test.cpp
  )
  if(TARGET online_path_following_controller_closed_loop_test)
    target_include_directories(online_path_following_controller_closed_loop_test
      PRIVATE
        src
    )
    target_link_libraries(online_path_following_controller_closed_loop_test
      online_path_following_controller_3d_core
    )
  endif()

  ament_add_gtest(online_path_following_controller_closed_loop_playback_test
    test/online_path_following_controller_closed_loop_playback_test.cpp
  )
  if(TARGET online_path_following_controller_closed_loop_playback_test)
    target_include_directories(online_path_following_controller_closed_loop_playback_test
      PRIVATE
        src
    )
    target_link_libraries(online_path_following_controller_closed_loop_playback_test
      online_path_following_controller_3d_core
    )
  endif()
endif()

ament_package()
