cmake_minimum_required(VERSION 3.16)
project(dubins_path_3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/types.cpp
  src/math_utils.cpp
  src/planar_dubins.cpp
  src/sphere_dubins.cpp
  src/cylinder_dubins.cpp
  src/surface_connections.cpp
  src/planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
# Eigen is public because it appears in the headers; threading is an
# implementation detail and must stay out of the exported link interface.
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE Threads::Threads)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_library(${PROJECT_NAME}_conversions INTERFACE)
target_include_directories(${PROJECT_NAME}_conversions INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}_conversions INTERFACE
  ${PROJECT_NAME}
  ${asr_sdm_control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS ${PROJECT_NAME}_conversions
  EXPORT export_${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # The closed-form expressions are transcribed from the reference paper, so the
  # copyright linter has nothing useful to say about them.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  foreach(test_name planar_dubins sphere_dubins cylinder_dubins planner msg_conversions)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    if(test_name STREQUAL msg_conversions)
      target_link_libraries(test_${test_name} ${PROJECT_NAME}_conversions)
    else()
      target_link_libraries(test_${test_name} ${PROJECT_NAME})
    endif()
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  eigen3_cmake_module
  asr_sdm_control_msgs
  geometry_msgs
)

ament_package()
