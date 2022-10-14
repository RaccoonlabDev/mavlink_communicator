ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO
LABEL description="Mavlink communicator"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/mavlink_communicator

# 1. Install basic requirements
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git python-catkin-tools python-pip python3-pip

# 2. Install requirements
COPY install_requirements.sh install_requirements.sh
RUN ./install_requirements.sh

# 3. Install dependencies
RUN git clone https://github.com/RaccoonlabDev/uavcan_msgs.git /catkin_ws/src/uavcan_msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

# 4. Copy the source files
COPY include/           include/
COPY src/               src/
COPY CMakeLists.txt     CMakeLists.txt
COPY package.xml        package.xml

# 5. Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

CMD source /opt/ros/melodic/setup.bash      && \
    source /catkin_ws/devel/setup.bash      && \
    echo "main process has been started"    && \
    echo "do nothing yet"                   && \
    echo "container has been finished"
