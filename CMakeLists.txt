cmake_minimum_required(VERSION 3.12)
project(my_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
message(STATUS "Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "CXX FLAGS: ${CMAKE_CXX_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

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

message(STATUS "------------- Find Sophus ----------------")
find_package(Sophus REQUIRED)
message(STATUS "Found Sophus Version: ${Sophus_VERSION}.")

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        src/Camera.cpp
        src/FeaturePoint.cpp
        src/Frame.cpp
        src/Map.cpp
        src/Config.cpp
        src/VisualOdometry.cpp
        src/g2oTypes.cpp
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${G2O_LIBRARIES})

add_executable(run_vo examples/run_vo.cpp)
target_link_libraries(run_vo ${PROJECT_NAME} ${OpenCV_LIBRARIES})

