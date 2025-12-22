cmake_minimum_required(VERSION 3.22.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(energy_predictor)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(
  energy_predictor_node_parameters # cmake target name for the parameter library
  src/energy_predictor_node_parameters.yaml # path to input yaml file
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_auto_add_library(least_squares SHARED
  DIRECTORY src
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/${PROJECT_NAME}
)

target_include_directories(
  least_squares PUBLIC
  include/${PROJECT_NAME}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${CERES_LIBRARIES}
  energy_predictor_node_parameters
)

target_link_libraries(
  least_squares
  Eigen3::Eigen
  ${CERES_LIBRARIES}
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN helios_cv::EnergyPredictorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(least_squares
  PLUGIN helios_cv::LeastSquaresNode
  EXECUTABLE least_squares_node
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

ament_auto_package()