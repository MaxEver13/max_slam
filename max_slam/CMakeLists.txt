cmake_minimum_required(VERSION 2.8.10)
project(max_slam)

message("Compiler version is: ${CMAKE_SYSTEM_VERSION}")
message("System version is: ${CMAKE_SYSTEM_NAME}")
message("Architecture is: ${CMAKE_SYSTEM_PROCESSOR}")

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(CMAKE_CXX_COMPILER clang++)
    set(CMAKE_C_COMPILER clang)
    set(MACOSX TRUE)
endif ()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
    set(OpenMP_LIBS gomp)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
endif ()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    pcl_conversions
    sensor_msgs
    tf
    std_msgs
    geometry_msgs
    message_generation
    cv_bridge
    image_transport
    ndt_omp # ndt_omp
    linefit_ground_segmentation # 地面分割
)


# loop info现在暂时没用
add_message_files(FILES FloorCoeffs.msg)


link_directories(${PCL_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
message("PCL Libraries include: ${PCL_LIBRARIES}")
message("PCL version: ${PCL_VERSION}")


# 地面分割
add_executable(ground_seg_node src/ground_segmentation_node.cpp)
target_link_libraries(ground_seg_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} linefit_ground_segmentation)



