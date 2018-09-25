cmake_minimum_required(VERSION 3.12)
project(my_slam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

message(STATUS "Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

message(STATUS "--------------- Find OpenCV ----------------")
find_package(OpenCV 3.4 REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV Version: ${OpenCV_VERSION}.")
else ()
    message(FATAL_ERROR "OpenCV > 3.4 not found.")
endif()

message(STATUS "--------------- Find Eigen -----------------")
find_package(Eigen3 3.3 REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Sophus Version: ${Eigen3_VERSION}.")
else()
    message(FATAL_ERROR "Eigen > 3.3 not found.")
endif()

message(STATUS "--------------- Find Sophus ----------------")
find_package(Sophus 1.0.0 REQUIRED)
if (Sophus_FOUND)
    message(STATUS "Found Sophus Version: ${Sophus_VERSION}.")
else()
    message(FATAL_ERROR "Sophus > 1.0.0 not found.")
endif()

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_LIBS SHARED
        src/Camera.cpp
)

