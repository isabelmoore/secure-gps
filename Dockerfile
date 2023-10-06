# Start with ROS 2 Humble full desktop image
FROM osrf/ros:humble-desktop-full

# Args for User
ARG UNAME=user
ARG UID=1000
ARG GID=1000

# Ensure that installs are non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install setup utils and basic dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        iputils-ping \
        udev \
        usbutils \
        net-tools \
        wget \
        iproute2 \
        curl \
        nano \
        git \
        python3-pip \
        ros-humble-rqt* \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf-transformations \
        ros-humble-navigation2 \
	    ros-humble-nav2-bringup \
	    ros-humble-turtlebot3* \
        ros-humble-velodyne-description \
        ros-humble-plotjuggler \
        ros-humble-plotjuggler-ros \
     && apt purge -y --auto-remove \
     && rm -rf /var/lib/apt/lists/*
     
# Python3 Packages required by task allocation
RUN pip3 install \
    numpy \
    matplotlib \
    transforms3d \
    utm

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $UNAME

# Allow the user to run sudo without a password
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the non-root user for other images
USER $UNAME

# Install Dataspeed SDK (https://bitbucket.org/DataspeedInc/dbw_ros/src/ros2/)
RUN /bin/bash -c "bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw_fca/scripts/sdk_install.bash)"

# Get extra dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-dataspeed-dbw-gazebo

# Get Workspace Dependencies
RUN mkdir -p ~/MKZ_SIMULATOR_PROTOTYPE1/src
COPY src /home/user/MKZ_SIMULATOR_PROTOTYPE1/src
RUN cd ~/MKZ_SIMULATOR_PROTOTYPE1 && \
    sudo apt update &&\
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Copy entrypoint
COPY docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]