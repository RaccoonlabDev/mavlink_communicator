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
 * @file mavlink_communicator.cpp
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 * @author Roman Fedorenko <frontwise@gmail.com>
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#include "mavlink_communicator.h"
#include <ros/ros.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <poll.h>
#include <netinet/tcp.h>

const std::string NODE_NAME = "Mavlink PX4 Communicator";

int MavlinkCommunicator::Init(int portOffset, bool is_copter_airframe){
    isCopterAirframe_ = is_copter_airframe;

    normalDistribution_ = std::normal_distribution<double>(0.0f, 0.1f);

    magNoise_ = 0.0000051;
    baroAltNoise_ = 0.0001;
    tempNoise_ = 0.001;
    absPressureNoise_ = 0.001;
    // Let's use a rough guess of 0.01 hPa as the standard devitiation which roughly yields
    // about +/- 1 m/s noise.
    diffPressureNoise_ = 0.01;

    memset((char *) &simulatorMavlinkAddr_, 0, sizeof(simulatorMavlinkAddr_));
    memset((char *) &px4MavlinkAddr_, 0, sizeof(px4MavlinkAddr_));
    simulatorMavlinkAddr_.sin_family = AF_INET;
    simulatorMavlinkAddr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    simulatorMavlinkAddr_.sin_port = htons(PORT_BASE + portOffset);

    if ((listenMavlinkSock_ = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        ROS_ERROR_STREAM(NODE_NAME << ": Creating TCP socket failed: " << strerror(errno));
        return -1;
    }

    // do not accumulate messages by waiting for ACK
    int yes = 1;
    int result = setsockopt(listenMavlinkSock_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }

    // try to close as fast as posible
    struct linger nolinger;
    nolinger.l_onoff = 1;
    nolinger.l_linger = 0;
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }

    // The socket reuse is necessary for reconnecting to the same address
    // if the socket does not close but gets stuck in TIME_WAIT. This can happen
    // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
    int socket_reuse = 1;
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }

    // Same as above but for a given port
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }


    if (bind(listenMavlinkSock_, (struct sockaddr *)&simulatorMavlinkAddr_, sizeof(simulatorMavlinkAddr_)) < 0){
        ROS_ERROR_STREAM(NODE_NAME << ": bind failed: " << strerror(errno));
    }

    errno = 0;
    result = listen(listenMavlinkSock_, 5);
    if (result < 0){
        ROS_ERROR_STREAM(NODE_NAME << ": listen failed: " << strerror(errno));
    }

    unsigned int px4_addr_len = sizeof(px4MavlinkAddr_);
    ROS_INFO_STREAM(NODE_NAME << ": waiting for connection from PX4...");
    std::cout << std::flush;
    while(true) {
        px4MavlinkSock_ = accept(listenMavlinkSock_,
                                (struct sockaddr *)&px4MavlinkAddr_,
                                &px4_addr_len);
        if (px4MavlinkSock_ < 0){
            ROS_ERROR_STREAM(NODE_NAME << ": accept failed: " << strerror(errno));
        }else{
            ROS_INFO_STREAM(NODE_NAME << ": PX4 Connected.");
            break;
        }
        std::cout << std::flush;
    }

    return result;
}


int MavlinkCommunicator::Clean(){
    close(px4MavlinkSock_);
    close(listenMavlinkSock_);
    return 0;
}


/**
 * @return result
 * -1 means error,
 * 0 means ok
 */
int MavlinkCommunicator::SendHilSensor(unsigned int time_usec,
                                       float gpsAltitude,
                                       Eigen::Vector3d magFrd,
                                       Eigen::Vector3d accFrd,
                                       Eigen::Vector3d gyroFrd,
                                       float staticPressure,
                                       float staticTemperature,
                                       float diffPressureHPa){
    // Output data
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = time_usec;

    // 1. Fill acc and gyro in FRD frame
    sensor_msg.xacc = accFrd[0];
    sensor_msg.yacc = accFrd[1];
    sensor_msg.zacc = accFrd[2];
    sensor_msg.xgyro = gyroFrd[0];
    sensor_msg.ygyro = gyroFrd[1];
    sensor_msg.zgyro = gyroFrd[2];
    sensor_msg.fields_updated = SENS_ACCEL | SENS_GYRO;

    // 2. Fill Magnetc field with noise
    if (time_usec - lastMagTimeUsec_ > MAG_PERIOD_US){
        sensor_msg.xmag = magFrd[0];
        sensor_msg.ymag = magFrd[1];
        sensor_msg.zmag = magFrd[2];
        sensor_msg.fields_updated |= SENS_MAG;
        lastMagTimeUsec_ = time_usec;
    }

    // 3. Fill Barometr and diff pressure
    if (time_usec - lastBaroTimeUsec_ > BARO_PERIOD_US){
        sensor_msg.temperature = staticTemperature;
        sensor_msg.abs_pressure = staticPressure;
        sensor_msg.pressure_alt = gpsAltitude;
        sensor_msg.pressure_alt += baroAltNoise_ * normalDistribution_(randomGenerator_);
        sensor_msg.diff_pressure = diffPressureHPa;

        sensor_msg.fields_updated |= SENS_BARO | SENS_DIFF_PRESS;
        lastBaroTimeUsec_ = time_usec;
    }


    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

    if(send(px4MavlinkSock_, buffer, packetlen, 0) != packetlen){
        return -1;
    }
    return 0;
}


/**
 * @return result
 * -1 means error,
 * 0 means ok
 */
int MavlinkCommunicator::SendHilGps(unsigned int time_usec,
                                    Eigen::Vector3d linearVelNed,
                                    Eigen::Vector3d gpsPosition){
    // Fill gps msg
    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = time_usec;
    hil_gps_msg.fix_type = 3;
    hil_gps_msg.lat = gpsPosition.x() * 1e7;
    hil_gps_msg.lon = gpsPosition.y() * 1e7;
    hil_gps_msg.alt = gpsPosition.z() * 1000;
    hil_gps_msg.eph = 100;
    hil_gps_msg.epv = 100;
    hil_gps_msg.vn = linearVelNed.x() * 100;
    hil_gps_msg.ve = linearVelNed.y() * 100;
    hil_gps_msg.vd = linearVelNed.z() * 100;
    hil_gps_msg.vel = std::sqrt(hil_gps_msg.vn * hil_gps_msg.vn + hil_gps_msg.ve * hil_gps_msg.ve);

    // Course over ground
    double cog = -std::atan2(hil_gps_msg.vn, hil_gps_msg.ve) * 180 / 3.141592654 + 90;

    if (cog < 0) {
        cog += 360;
    }
    hil_gps_msg.cog = cog * 100;
    hil_gps_msg.satellites_visible = 10;

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    int packetlen = mavlink_msg_to_send_buffer(buffer, &msg);
    if(packetlen == 0 || send(px4MavlinkSock_, buffer, packetlen, 0) != packetlen){
        return -1;
    }
    return 0;
}


/**
 * @return status
 * -1 means error,
 * 0 means there is no rx command
 * 1 means there is an actuator command
 */
int MavlinkCommunicator::Receive(bool blocking, bool &armed, std::vector<double>& command){
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    struct pollfd fds[1] = {};
    fds[0].fd = px4MavlinkSock_;
    fds[0].events = POLLIN;

    int p = poll(&fds[0], 1, (blocking?-1:2));
    if(p < 0){
        return -1;
    }else if(p == 0){
        return 0;
    }else if(fds[0].revents & POLLIN){
        unsigned int slen = sizeof(px4MavlinkAddr_);
        unsigned int len = recvfrom(px4MavlinkSock_,
                                    buffer,
                                    sizeof(buffer),
                                    0,
                                    (struct sockaddr *)&px4MavlinkAddr_,
                                    &slen);
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i){
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)){
                if(msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){
                    mavlink_hil_actuator_controls_t controls;
                    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                    armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    if(armed){
                        command[0] = controls.controls[0];
                        command[1] = controls.controls[1];
                        command[2] = controls.controls[2];
                        command[3] = controls.controls[3];
                        if(isCopterAirframe_ == false){
                            command[4] = controls.controls[4];
                            command[5] = controls.controls[5];
                            command[6] = controls.controls[6];
                            command[7] = controls.controls[7];
                        }
                    }
                    return 1;
                }else if (msg.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS){
                    ROS_ERROR_STREAM_THROTTLE(2, NODE_NAME << ": MAVLINK_MSG_ID_ESTIMATOR_STATUS");
                }else{
                    ROS_WARN_STREAM(NODE_NAME << ": unknown msg with msgid = " << msg.msgid);
                }
            }
        }
        ROS_WARN_STREAM_THROTTLE(1, NODE_NAME << ": No cmd");
        return 0;
    }
    return -1;
}
