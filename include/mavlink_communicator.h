/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o.. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_communicator.h
 *
 * @author Roman Fedorenko <frontwise@gmail.com>
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#ifndef PX4_COMMUNICATOR_H
#define PX4_COMMUNICATOR_H

#include <ros/ros.h>
#include <netinet/in.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <uavcan_msgs/Fix.h>
#include <uavcan_msgs/StaticPressure.h>
#include <uavcan_msgs/RawAirData.h>


class MavlinkCommunicator{
public:
    MavlinkCommunicator() {};

    /**
     * @brief Init connection with PX4 using TCP
     * @param is_copter_airframe - input
     * Mavlink actuator msg say nothing about is it VTOL or copter and it is always has 8 channels,
     * so we can't understand airframe in real time.
     * The problem is that in copter airframe mavlink fill last 4 channels by random values,
     * so we should initially manually define airframe and just fill last 4 channels by zeros.
     * @note Be sure if your VTOL mixer has shifted values in control surface, for example
     * aileron from 0 to 1, where 0.5 is default - in copter mode communicator fill it by zeros.
     */
    int Init(int portOffset, bool is_copter_airframe);

    /**
     * @brief Close mavlink sockets
     */
    int Clean();

    /**
     * @brief Send hil_sensor (#107) and hil_gps (#113) to PX4 via mavlink
     */
    int SendHilSensor(unsigned int time_usec,
                      float gpsAltitude,
                      Eigen::Vector3d mag_frd,
                      Eigen::Vector3d acc_frd,
                      Eigen::Vector3d gyro_frd,
                      float staticTemperature,
                      float staticPressure,
                      float diffPressure);
    int SendHilGps(unsigned int time_usec,
                   Eigen::Vector3d vel_ned,
                   Eigen::Vector3d pose_geodetic);


    /**
     * @brief Receive hil_actuator_controls (#93) from PX4 via mavlink
     * @param blocking - input
     * @param armed - output
     * @param command - output
     */
    int Receive(bool blocking, bool &armed, std::vector<double>& command);

private:
    bool isCopterAirframe_;

    static const uint64_t SENS_ACCEL       = 0b111;
    static const uint64_t SENS_GYRO        = 0b111000;
    static const uint64_t SENS_MAG         = 0b111000000;
    static const uint64_t SENS_BARO        = 0b1101000000000;
    static const uint64_t SENS_DIFF_PRESS  = 0b10000000000;

    // const float ALT_HOME;

    static constexpr uint64_t MAG_PERIOD_US = 1e6 / 100;
    static constexpr uint64_t BARO_PERIOD_US = 1e6 / 50;
    uint64_t lastMagTimeUsec_ = 0;
    uint64_t lastBaroTimeUsec_ = 0;

    const int PORT_BASE = 4560;
    struct sockaddr_in px4MavlinkAddr_;
    struct sockaddr_in simulatorMavlinkAddr_;
    int listenMavlinkSock_;
    int px4MavlinkSock_;

    std::default_random_engine randomGenerator_;
    std::normal_distribution<double> normalDistribution_;
    double magNoise_;
    double baroAltNoise_;
    double tempNoise_;
    double absPressureNoise_;
    double diffPressureNoise_;
};


class MavlinkCommunicatorROS
{
public:
    explicit MavlinkCommunicatorROS(ros::NodeHandle nodeHandler, float lat_home);
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
    uavcan_msgs::StaticPressure staticPressureMsg_;
    float staticPressure_;
    void staticPressureCallback(uavcan_msgs::StaticPressure::Ptr staticPressure);

    ros::Subscriber rawAirDataSub_;
    uavcan_msgs::RawAirData rawAirDataMsg_;
    float diffPressure_;
    void rawAirDataCallback(uavcan_msgs::RawAirData::Ptr rawAirData);

    ros::Subscriber gpsSub_;
    uavcan_msgs::Fix gpsPositionMsg_;
    Eigen::Vector3d gpsPosition_;
    Eigen::Vector3d linearVelocityNed_;
    uint64_t gpsMsgCounter_ = 0;
    void gpsCallback(uavcan_msgs::Fix::Ptr gpsPosition);

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


#endif
