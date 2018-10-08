cmake_minimum_required(VERSION 3.12)
project(my_slam)

set(CMAKE_CXX_STANDARD 17)
message(STATUS "Standard: C++${CMAKE_CXX_STANDARD}")

message(STATUS "--------------- Find OpenCV ----------------")
find_package(OpenCV 3.4 REQUIRED)
message(STATUS "Found OpenCV Version: ${OpenCV_VERSION}.")

message(STATUS "--------------- Find Eigen -----------------")
find_package(Eigen3 3.3 REQUIRED)
message(STATUS "Found Sophus Version: ${Eigen3_VERSION}.")

message(STATUS "--------------- Find Ceres -----------------")
find_package(Ceres 1.10.0 REQUIRED)
message(STATUS "Found Ceres Version: ${Ceres_VERSION}.")

message(STATUS "---------------- Find G2O ------------------")
find_package(G2O 1.0.0 REQUIRED)
message(STATUS "Found G2O Version : ${G2O_VERSION}.")
set(G2O_LIBRARIES
        ${G2O_STUFF_LIBRARY}
        ${G2O_CORE_LIBRARY}
        ${G2O_CLI_LIBRARY}
        ${G2O_SOLVER_CHOLMOD}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
        ${G2O_SOLVER_DENSE}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_SLAM2D_LINEAR}
        ${G2O_SOLVER_STRUCTURE_ONLY}
        ${G2O_TYPES_DATA}
        ${G2O_TYPES_ICP}
        ${G2O_TYPES_SBA}
        ${G2O_TYPES_SCLAM2D}
        ${G2O_TYPES_SIM3}
        ${G2O_TYPES_SLAM2D}
        ${G2O_TYPES_SLAM3D}
)

message(STATUS "------------- Find yaml-cpp ----------------")
find_package(yaml-cpp REQUIRED)
message(STATUS "Found yaml-cpp Version: ${yaml-cpp_VERSION}.")

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
        src/Camera.cpp
        src/FeaturePoint.cpp
        src/Frame.cpp
        src/Map.cpp
        src/Config.cpp
        src/VisualOdometry.cpp
)

add_executable(run_vo examples/run_vo.cpp)
target_link_libraries(run_vo ${PROJECT_NAME})

