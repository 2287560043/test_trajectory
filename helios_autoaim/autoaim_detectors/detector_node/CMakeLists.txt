cmake_minimum_required(VERSION 3.22.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(detector_node)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
	find_package(OpenVINO REQUIRED)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("this is arm64 cpu")
endif()

ament_auto_find_build_dependencies()

generate_parameter_library(
  detector_node_parameters # cmake target name for the parameter library
  src/detector_node_parameters.yaml # path to input yaml file
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    detector_node_parameters
    openvino::runtime
  )
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    detector_node_parameters
  )
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN helios_cv::DetectorNode
  EXECUTABLE ${PROJECT_NAME}_node
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