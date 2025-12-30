cmake_minimum_required(VERSION 3.22.0)
project(camera_imu_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(MVSDK_ROOT "${PROJECT_SOURCE_DIR}/lib/MindVisionSDK" 
    CACHE PATH "MindVision SDK root directory")
set(MVSDK_INCLUDE_DIR "${MVSDK_ROOT}/include")
set(MVSDK_LIB_DIR "${MVSDK_ROOT}/lib")
set(FTD2XX_ROOT "${PROJECT_SOURCE_DIR}/lib/ftd2xx" 
    CACHE PATH "ftd2xx root directory")
set(FTD2XX_INCLUDE_DIR "${FTD2XX_ROOT}/include")
set(FTD2XX_LIB_DIR "${FTD2XX_ROOT}/lib")

find_library(MVSDK_LIBRARY
  NAMES MVSDK
  PATHS ${MVSDK_LIB_DIR}
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(FTD2XX_LIBRARY
    NAMES ftd2xx
    PATHS ${FTD2XX_LIB_DIR}
  NO_DEFAULT_PATH
  REQUIRED
)

generate_parameter_library(camera_parameters
  config/camera_imu_bridge_params.yaml
)

ament_auto_add_library(camera_imu_bridge SHARED
    src/CameraImuBridgeNode.cpp
    src/Camera.cpp
    src/ImuFactory.cpp
    src/ImuSerial.cpp
    src/ImuFtd2xx.cpp
    src/Syncer.cpp
)

target_include_directories(camera_imu_bridge PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(camera_imu_bridge PUBLIC
    ${FTD2XX_INCLUDE_DIR}
)
target_include_directories(camera_imu_bridge PUBLIC
  ${MVSDK_INCLUDE_DIR}
)

target_link_libraries(camera_imu_bridge
    ${MVSDK_LIBRARY}
    ${FTD2XX_LIBRARY}
    camera_parameters
    ${OpenCV_LIBS}
)

set_target_properties(camera_imu_bridge PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
)

rclcpp_components_register_node(camera_imu_bridge
  PLUGIN "helios_cv::CameraImuBridgeNode"
  EXECUTABLE camera_imu_bridge_node
)
install(
  DIRECTORY lib/MindVisionSDK
  DESTINATION lib
)
install(
  DIRECTORY lib/ftd2xx
  DESTINATION lib
)

install(
  DIRECTORY config
  DESTINATION share/${project_name}
)

ament_auto_package()

