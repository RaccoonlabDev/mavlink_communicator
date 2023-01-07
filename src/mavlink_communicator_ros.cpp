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
 * @file mavlink_communicator.cpp
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include "mavlink_communicator_ros.h"


const std::string NODE_NAME = "Mavlink PX4 Communicator";
constexpr char ACTUATOR_TOPIC_NAME[]            = "/uav/actuators";
constexpr char ARM_TOPIC_NAME[]                 = "/uav/arm";

constexpr char STATIC_TEMPERATURE_TOPIC_NAME[]  = "/uav/static_temperature";
constexpr char STATIC_PRESSURE_TOPIC_NAME[]     = "/uav/static_pressure";
constexpr char DIFF_PRESSURE_TOPIC_NAME[]       = "/uav/raw_air_data";
constexpr char GPS_POINT_TOPIC_NAME[]           = "/uav/gps_point";
constexpr char VELOCITY_TOPIC_NAME[]            = "/uav/velocity";
constexpr char IMU_TOPIC_NAME[]                 = "/uav/imu";
constexpr char MAG_TOPIC_NAME[]                 = "/uav/mag";


MavlinkCommunicatorROS::MavlinkCommunicatorROS(ros::NodeHandle nodeHandler) :
    nodeHandler_(nodeHandler){
}

int MavlinkCommunicatorROS::Init(int portOffset, bool is_copter_airframe){
    int result = mavlinkCommunicator_.Init(portOffset, is_copter_airframe);
    if(result != 0){
        return result;
    }

    armPub_ = nodeHandler_.advertise<std_msgs::Bool>(ARM_TOPIC_NAME, 1);
    actuatorsPub_ = nodeHandler_.advertise<sensor_msgs::Joy>(ACTUATOR_TOPIC_NAME, 1);


    staticTemperatureSub_ = nodeHandler_.subscribe(STATIC_TEMPERATURE_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::staticTemperatureCallback,
        this);

    staticPressureSub_ = nodeHandler_.subscribe(STATIC_PRESSURE_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::staticPressureCallback,
        this);

    diffPressurePaSub_ = nodeHandler_.subscribe(DIFF_PRESSURE_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::diffPressureCallback,
        this);

    gpsPositionSub_ = nodeHandler_.subscribe(GPS_POINT_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::gpsPositionCallback,
        this);
    gpsVelocitySub_ = nodeHandler_.subscribe(VELOCITY_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::gpsVelocityCallback,
        this);
    imuSub_ = nodeHandler_.subscribe(IMU_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::imuCallback,
        this);
    magSub_ = nodeHandler_.subscribe(MAG_TOPIC_NAME,
        1,
        &MavlinkCommunicatorROS::magCallback,
        this);

    return result;
}

void MavlinkCommunicatorROS::communicate(){
    auto gpsTimeUsec = gpsPositionMsg_.header.stamp.toNSec() / 1000;
    auto imuTimeUsec = imuMsg_.header.stamp.toNSec() / 1000;

    std::vector<double> actuators(8);
    if(mavlinkCommunicator_.Receive(false, isArmed_, actuators) == 1){
        publishActuators(actuators);
        publishArm();
    }

    /**
     * @note For some reasons sometimes PX4 ignores GPS all messages after first if we
     * send it too soon. So, just ignoring first few messages is ok.
     * @todo Understand why and may be develop a better approach
     */
    if (gpsMsgCounter_ >= 5 && gpsTimeUsec >= lastGpsTimeUsec_ + GPS_PERIOD_US){
        lastGpsTimeUsec_ = gpsTimeUsec;

        if(mavlinkCommunicator_.SendHilGps(gpsTimeUsec, linearVelocityNed_, gpsPosition_) == -1){
            ROS_ERROR_STREAM_THROTTLE(1, NODE_NAME << ": GPS failed." << strerror(errno));
        }
    }
    if (imuTimeUsec >= lastImuTimeUsec_ + IMU_PERIOD_US){
        lastImuTimeUsec_ = imuTimeUsec;

        int status = mavlinkCommunicator_.SendHilSensor(imuTimeUsec,
                                                        gpsPosition_.z(),
                                                        magFrd_,
                                                        accFrd_,
                                                        gyroFrd_,
                                                        staticPressure_,
                                                        staticTemperature_,
                                                        diffPressureHPa_);

        if(status == -1){
            ROS_ERROR_STREAM_THROTTLE(1, NODE_NAME << "Imu failed." << strerror(errno));
        }
    }
}

void MavlinkCommunicatorROS::publishArm(){
    std_msgs::Bool armMsg;
    armMsg.data = isArmed_;
    armPub_.publish(armMsg);
}

void MavlinkCommunicatorROS::publishActuators(const std::vector<double>& actuators) const{
    // it's better to move it to class members to prevent initialization on each publication
    sensor_msgs::Joy actuatorsMsg;
    actuatorsMsg.header.stamp = ros::Time::now();
    for(auto actuator : actuators){
        actuatorsMsg.axes.push_back(actuator);
    }
    actuatorsPub_.publish(actuatorsMsg);
}

void MavlinkCommunicatorROS::staticTemperatureCallback(std_msgs::Float32::Ptr msg){
    staticTemperatureMsg_ = *msg;
    staticTemperature_ = msg->data - 273.15;
}

void MavlinkCommunicatorROS::staticPressureCallback(std_msgs::Float32::Ptr msg){
    staticPressureMsg_ = *msg;
    staticPressure_ = msg->data / 100;
}

void MavlinkCommunicatorROS::diffPressureCallback(std_msgs::Float32::Ptr msg){
    diffPressurePaMsg_ = *msg;
    diffPressureHPa_ = msg->data / 100;
}

void MavlinkCommunicatorROS::gpsPositionCallback(sensor_msgs::NavSatFix::Ptr msg){
    gpsPositionMsg_ = *msg;
    gpsMsgCounter_++;
    gpsPosition_[0] = msg->latitude;
    gpsPosition_[1] = msg->longitude;
    gpsPosition_[2] = msg->altitude;
}


void MavlinkCommunicatorROS::gpsVelocityCallback(geometry_msgs::Twist::Ptr msg){
    gpsVelocityMsg_ = *msg;
    linearVelocityNed_[0] = msg->linear.x;
    linearVelocityNed_[1] = msg->linear.y;
    linearVelocityNed_[2] = msg->linear.z;
}

void MavlinkCommunicatorROS::imuCallback(sensor_msgs::Imu::Ptr imu){
    imuMsg_ = *imu;
    accFrd_[0] = imu->linear_acceleration.x;
    accFrd_[1] = imu->linear_acceleration.y;
    accFrd_[2] = imu->linear_acceleration.z;

    gyroFrd_[0] = imu->angular_velocity.x;
    gyroFrd_[1] = imu->angular_velocity.y;
    gyroFrd_[2] = imu->angular_velocity.z;
}

void MavlinkCommunicatorROS::magCallback(sensor_msgs::MagneticField::Ptr mag){
    magMsg_ = *mag;
    magFrd_[0] = mag->magnetic_field.x;
    magFrd_[1] = mag->magnetic_field.y;
    magFrd_[2] = mag->magnetic_field.z;
}

int main(int argc, char **argv){
    // 1. Init node
    ros::init(argc, argv, NODE_NAME.c_str());
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle nodeHandler("mavlink_communicator");

    // 2. Define which mavlink actuators format should we use (
    // - quad rotors with actuators cmd size = 4
    // - or VTOL with actuators cmd size = 8)
    std::string vehicle;
    if(!nodeHandler.getParam("vehicle", vehicle)){
        ROS_ERROR_STREAM(NODE_NAME << ": " << "vehicle" << " parameter is missing.");
        ros::shutdown();
    }
    bool isCopterAirframe;
    const std::string VEHICLE_IRIS = "iris";
    const std::string VEHICLE_INNOPOLIS_VTOL = "innopolis_vtol";
    if(vehicle == VEHICLE_INNOPOLIS_VTOL){
        isCopterAirframe = false;
    }else if(vehicle == VEHICLE_IRIS){
        isCopterAirframe = true;
    }else{
        ROS_ERROR_STREAM(NODE_NAME << ": " << "vehicle" << " parameter is wrong.");
        ros::shutdown();
    }

    int px4id = 0;

    MavlinkCommunicatorROS communicator(nodeHandler);
    if(communicator.Init(px4id, isCopterAirframe) != 0) {
        ROS_ERROR("Unable to Init PX4 Communication");
        ros::shutdown();
    }

    ros::Rate r(500);
    while(ros::ok()){
        communicator.communicate();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
