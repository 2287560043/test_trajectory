cmake_minimum_required(VERSION 3.22.0)

project(camera_imu_syncer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)


ament_auto_find_build_dependencies()

add_subdirectory(lib/MVSDK)

ament_auto_add_executable(camera_imu_syncer_node
  src/camera_imu_syncer.cpp
  src/Camera.cpp
  src/Imu.cpp
  src/Syncer.cpp
)


target_include_directories(
  camera_imu_syncer_node PUBLIC
  include/camera_imu_syncer_node
)

target_link_libraries(camera_imu_syncer_node
  mindvision_camera
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
)




