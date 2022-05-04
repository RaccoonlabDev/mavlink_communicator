cmake_minimum_required(VERSION 3.0.2)
project(mavlink_communicator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    uavcan_msgs
    tf2
    tf2_ros
)

find_package(mavlink)
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs uavcan_msgs
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} STATIC src/mavlink_communicator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)

## Build mavlink_communicator node
add_executable(${PROJECT_NAME}_node src/mavlink_communicator.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)