cmake_minimum_required(VERSION 3.0.2)
project(mavlink_communicator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)

find_package(mavlink)
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

# MAVLink shows many warnings. Let's just ignore them for a while
# https://stackoverflow.com/questions/65011913/cmake-suppress-all-warnings
add_definitions(-w)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} STATIC
    src/mavlink_communicator_ros.cpp
    src/mavlink_communicator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

## Build mavlink_communicator node
add_executable(${PROJECT_NAME}_node src/mavlink_communicator_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)