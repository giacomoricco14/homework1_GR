FROM osrf/ros:humble-desktop

#Install essential
RUN apt-get update && apt-get install -y

#Gazebo ignition
RUN apt-get install -y lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME=/home/user
ENV ROS_DISTRO=humble

#Add non root user using UID and GID passed as argument
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers
USER user

#ROS2 workspace creation and compilation
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws/
#COPY --chown=user ./src ${HOME}/ros2_ws/src
RUN rosdep update
USER root
RUN rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO} -y 
USER user
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install

#Add script and export env variables to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ${HOME}/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ${HOME}/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${HOME}/.bashrc
RUN echo "export GAZEBO_AUDIO=0" >> ${HOME}/.bashrc
RUN echo "export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/src/armando_gazebo/models" >> ${HOME}/.bashrc

USER root
RUN apt-get upgrade -y && apt-get update -y

RUN apt-get install -y ros-humble-ros-ign-bridge && \
apt-get install -y ros-humble-ros-gz && \
apt-get install ros-humble-controller-manager -y && \
apt-get install ros-humble-ros2-control -y && \
apt-get install ros-humble-ros2-controllers -y && \
apt-get install ros-humble-ign-ros2-control -y && \
apt-get install ros-humble-urdf-tutorial -y && \
apt-get install ros-humble-xacro -y && \
apt-get install ros-humble-ros-gz-sim -y

#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
#RUN chown root:user /dev/video0
USER user
