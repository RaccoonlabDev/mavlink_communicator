/* 
 * Copyright (c) 2020-2022 RaccoonLab.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

/**
 * @file mavlink_communicator_ros.h
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#ifndef PX4_COMMUNICATOR_ROS_H
#define PX4_COMMUNICATOR_ROS_H

#include "mavlink_communicator.h"
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>


class MavlinkCommunicatorROS
{
public:
    explicit MavlinkCommunicatorROS(ros::NodeHandle nodeHandler);
    int Init(int portOffset, bool is_copter_airframe);
    void communicate();

private:
    MavlinkCommunicator mavlinkCommunicator_;
    ros::NodeHandle nodeHandler_;

    ros::Publisher actuatorsPub_;
    void publishActuators(const std::vector<double>& actuators) const;
    ros::Publisher armPub_;
    bool isArmed_;
    void publishArm();

    ros::Subscriber staticTemperatureSub_;
    std_msgs::Float32 staticTemperatureMsg_;
    float staticTemperature_;
    void staticTemperatureCallback(std_msgs::Float32::Ptr staticTemperature);

    ros::Subscriber staticPressureSub_;
    std_msgs::Float32 staticPressureMsg_;
    float staticPressure_;
    void staticPressureCallback(std_msgs::Float32::Ptr staticPressure);

    ros::Subscriber diffPressurePaSub_;
    std_msgs::Float32 diffPressurePaMsg_;
    float diffPressureHPa_;
    void diffPressureCallback(std_msgs::Float32::Ptr diffPressurePaMsg);

    uint64_t gpsMsgCounter_ = 0;

    ros::Subscriber gpsPositionSub_;
    sensor_msgs::NavSatFix gpsPositionMsg_;
    Eigen::Vector3d gpsPosition_;
    void gpsPositionCallback(sensor_msgs::NavSatFix::Ptr gpsPositionMsg);

    ros::Subscriber gpsVelocitySub_;
    geometry_msgs::Twist gpsVelocityMsg_;
    Eigen::Vector3d linearVelocityNed_;
    void gpsVelocityCallback(geometry_msgs::Twist::Ptr gpsVelocityMsg);

    ros::Subscriber imuSub_;
    sensor_msgs::Imu imuMsg_;
    Eigen::Vector3d accFrd_;
    Eigen::Vector3d gyroFrd_;
    void imuCallback(sensor_msgs::Imu::Ptr imu);

    ros::Subscriber magSub_;
    sensor_msgs::MagneticField magMsg_;
    Eigen::Vector3d magFrd_;
    void magCallback(sensor_msgs::MagneticField::Ptr mag);

    static constexpr uint64_t GPS_PERIOD_US = 1e6 / 10;
    static constexpr uint64_t IMU_PERIOD_US = 1e6 / 500;
    uint64_t lastGpsTimeUsec_ = 0;
    uint64_t lastImuTimeUsec_ = 0;
};


#endif  // PX4_COMMUNICATOR_ROS_H
