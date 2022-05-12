# adapted from: https://github.com/Tiryoh/docker-ros2-desktop-vnc
# and https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/ros2_docker/Dockerfile 
FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

# CoppeliaSim
RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        wget \
        vim tar xz-utils \
        libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
        xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
        libavcodec-dev libavformat-dev libswscale-dev \
        liblua5.3-0 \
        qtcreator \
        python3-pip \
        python3-tk \
        python3-opencv \
        ros-foxy-gazebo-msgs \
        xsltproc \
        qt5-default && \
    apt-get autoclean -y && apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*


COPY ros_pkg_ws ros_pkg_ws

ARG coppeliasim_version=CoppeliaSim_Pro_V4_1_0_Ubuntu20_04
RUN wget -q -P /sim http://coppeliarobotics.com/files/${coppeliasim_version}.tar.xz
#COPY ${coppeliasim_version}.tar.xz sim/

RUN tar -xf sim/${coppeliasim_version}.tar.xz -C sim/ && \
    rm sim/${coppeliasim_version}.tar.xz


# PyRep
ENV COPPELIASIM_ROOT_DIR=/sim/${coppeliasim_version}/
ENV COPPELIASIM_ROOT=${COPPELIASIM_ROOT_DIR}
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH
ENV PATH=$COPPELIASIM_ROOT_DIR:$PATH
ENV QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
RUN git clone https://github.com/stepjam/PyRep.git 
RUN cd PyRep && \
    pip3 install -r requirements.txt && \
    pip3 install xmlschema &&\
    pip3 install numpy && \
    pip3 install .


# The right libPlugin
RUN cd $COPPELIASIM_ROOT/programming && \
    rm -r libPlugin && \
    git clone https://github.com/CoppeliaRobotics/libPlugin.git && \
    cd libPlugin && \
    git checkout tags/coppeliasim-v4.1.0

#CoppeliaSim plugin for ROS2 simExtROS2Interface, for 4.1 
RUN git clone --recursive https://github.com/CoppeliaRobotics/simExtROS2Interface.git sim_ros2_interface && \
    cd sim_ros2_interface && \
    git checkout tags/coppeliasim-v4.1.0 && \
    . /opt/ros/$ROS_DISTRO/setup.bash && \
    echo $'sensor_msgs/msg/RegionOfInterest\nsensor_msgs/msg/CameraInfo' >> meta/interfaces.txt && \
    ulimit -s unlimited && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DLIBPLUGIN_DIR=$COPPELIASIM_ROOT/programming/libPlugin

RUN . /opt/ros/$ROS_DISTRO/setup.bash && \ 
    cd /ros_pkg_ws/ && \
    vcs import src < src/src_dependencies.rosinstall  && \ 
    rosdep install --default-yes --ignore-packages-from-source --from-path ./src  && \
    colcon build
    

RUN cp /sim_ros2_interface/build/sim_ros2_interface/libsimExtROS2Interface.so $COPPELIASIM_ROOT

RUN cd ${COPPELIASIM_ROOT} && \
    echo | ls -lh 

RUN chmod +x /ros_entrypoint.sh && \ 
    chmod +x $COPPELIASIM_ROOT/libsimExtROS2Interface.so

RUN cd ${COPPELIASIM_ROOT} && \
    echo | ls -lh 

COPY ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

# Powershell for: What ip??
# Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" | select -exp "IPAddress"
# $shakeip = Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" | select -exp "IPAddress" | out-string;
# docker run -it --rm -e DISPLAY=172.29.48.1:0.0 shakeit bin/bash 
# $shakeip = Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" | select -exp "IPAddress"; docker run -it --rm -e DISPLAY=${shakeip}:0.0 shakeit bin/bash
# ros2 launch shakeit_experiments run_sim_experiment.launch.py
# $shakeip = Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" | select -exp "IPAddress"; docker run -it --rm --name shakeit -e DISPLAY=${shakeip}:0.0 shakeit ros2 launch shakeit_experiments run_sim_experiment.launch.py

# Do this readme ubuntu_20.04 CUDA