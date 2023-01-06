# MAVLink communicator

MAVLink communicator is a bridge between MAVLink and ROS designed as ROS package.

The package covers a minimal set of sensors required for PX4 MAVLink SITL simulation. It is based on few MAVLink messages:

**1. [HIL_ACTUATOR_CONTROLS (MAVLink->ROS)](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS)**

| № | Default ROS Topic name | ROS msg         |
| - | ----------------------- | --------------- |
| 1 | /uav/actuators | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) |
| 2 | /uav/arm | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) |

**2. [HIL_SENSOR (ROS->MAVLink)](https://mavlink.io/en/messages/common.html#HIL_SENSOR)**

| № | Default ROS Topic name | ROS msg         |
| - | ----------------------- | --------------- |
| 1 | /uav/static_temperature | [std_msgs::Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) |
| 2 | /uav/static_pressure    | [std_msgs::Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) |
| 3 | /uav/raw_air_data       | [std_msgs::Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) |
| 4 | /uav/imu | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)  |
| 5 | /uav/mag | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html) |

**3. [HIL_GPS (ROS->MAVLink)](https://mavlink.io/en/messages/common.html#HIL_GPS)**

| № | Default ROS Topic name | ROS msg         |
| - | ----------------------- | --------------- |
| 1 | /uav/gps_position       | [sensor_msgs::NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) |
| 2 | /uav/velocity           | [geometry_msgs::Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) |

## Usage

One way is to build it as ROS package and call it as ROS node:

```bash
rosrun mavlink_communicator mavlink_communicator_node
```

Another way is to use is inside docker. Try [docker/docker.sh](docker/docker.sh) script. For example:

```bash
./docker/docker.sh --help
```

## Application example

This package is a part of [UAV HITL dynamics simulator](https://github.com/RaccoonlabDev/innopolis_vtol_dynamics).
