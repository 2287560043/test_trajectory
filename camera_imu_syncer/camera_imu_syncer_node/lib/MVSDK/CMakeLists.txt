cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MVSDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR})


if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(MVSDK_LIB_PATH ${MVSDK_ROOT}/lib/x64/libMVSDK.so)
else(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(MVSDK_LIB_PATH ${MVSDK_ROOT}/lib/arm64/libMVSDK.so)
endif()

if(NOT EXISTS ${MVSDK_LIB_PATH})
    message(FATAL_ERROR "MVSDK library not found: ${MVSDK_LIB_PATH}")
endif()

find_package(OpenCV REQUIRED)

set(MINDVISION_SOURCES MindVisionCamera.cpp)

add_library(mindvision_camera STATIC ${MINDVISION_SOURCES})
target_compile_options(mindvision_camera PRIVATE -Wno-unused-parameter)

target_include_directories(mindvision_camera
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${MVSDK_ROOT}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(mindvision_camera
    PUBLIC
        ${MVSDK_LIB_PATH}
        ${OpenCV_LIBS}
        pthread
        dl
)

message(STATUS "MVSDK library: ${MVSDK_LIB_PATH}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
