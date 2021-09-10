cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_segmentation)

# find_package(catkin_simple 0.1.0 REQUIRED )

# catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -g)

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES linefit_ground_segmentation
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ground_segmentation.cc src/segment.cc src/bin.cc)


############
## INSTAL ##
############

install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install headers
install(DIRECTORY include/ground_segmentation
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
