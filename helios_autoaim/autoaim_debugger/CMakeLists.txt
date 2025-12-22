cmake_minimum_required(VERSION 3.22.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(autoaim_debugger)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(
  autoaim_debugger_parameters # cmake target name for the parameter library
  src/autoaim_debugger_parameters.yaml # path to input yaml file
)


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/AutoAimDebugger.cpp
)

ament_auto_add_library(video_publisher SHARED
  src/VideoPublisher.cpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(
  video_publisher PUBLIC
  include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  autoaim_debugger_parameters
)

target_link_libraries(
  video_publisher
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN helios_cv::AutoAimDebugger
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(
  video_publisher
  PLUGIN helios_cv::VideoPublisher
  EXECUTABLE video_publisher_node
)


if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  )