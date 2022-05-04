# Mavlink communicator

Mavlink communicator is a bridge between Mavlink and ROS.

This package is a part of [Innopolis VTOL dynamics simulator](https://github.com/InnopolisAero/innopolis_vtol_dynamics).

It covers a minimal set of sensors required for such applications as PX4 UAVCAN HITL simulation.

The tables below represent the supported conversions:

**Mavlink->ROS**

| № | ROS Topic               | ROS msg         | MAVLink msg |
| - | ----------------------- | --------------- | ----------- |
| 1 | /uav/actuators | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) | [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) |
| 2 | /uav/actuators | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) | [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) |

**ROS->Mavlink**

| № | ROS Topic               | ROS msg         | MAVLink msg |
| - | ----------------------- | --------------- | ----------- |
| 1 | /uav/static_temperature | uavcan_msgs/StaticTemperature | [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) |
| 2 | /uav/static_pressure    | uavcan_msgs/StaticPressure    | [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) |
| 3 | /uav/raw_air_data       | uavcan_msgs/RawAirData        | [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) |
| 4 | /uav/gps_position       | uavcan_msgs/Fix | [HIL_GPS](https://mavlink.io/en/messages/common.html#HIL_GPS) |
| 5 | /uav/imu | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)  | [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) |
| 6 | /uav/mag | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html) | [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) |

Here we try to use default ROS messages as much as possible, but sometimes we need to define our own messages `uavcan_msgs` to be compatible with [UavcanCommunicator](https://github.com/InnopolisAero/uavcan_communicator).
