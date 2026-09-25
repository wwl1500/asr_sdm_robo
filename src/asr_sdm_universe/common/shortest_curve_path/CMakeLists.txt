cmake_minimum_required(VERSION 3.8)
project(shortest_curve_path)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/reeds_shepp.cpp
  src/spatial_reeds_shepp.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(spatial_reeds_shepp_test
    test/spatial_reeds_shepp_test.cpp
  )
  target_link_libraries(spatial_reeds_shepp_test
    ${PROJECT_NAME}
  )
  install(TARGETS spatial_reeds_shepp_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
