# Base image with ROS noetic (Python3.8 default)
FROM osrf/ros:noetic-desktop

# Create non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

# Add sudo and Python 3
RUN apt-get update && \
    apt-get install -y sudo python3-dev python3-pip && \
    # optional: make python3 the default
    update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install base packages
RUN apt-get update && apt-get install -y \
    git iputils-ping x11-apps sshfs sshpass net-tools \
    netcat openssh-server avahi-daemon libnss-mdns iproute2 \
    tmux vim nano curl

COPY yolo /home/ubuntu/yolo
COPY yolo/requirements.txt /requirements.txt
# yolov8
RUN pip3 install --no-cache-dir ultralytics
# jetson_camera, motion requirements
RUN pip3 install --no-cache-dir -r /requirements.txt


# Change ownership ---STEP 1
RUN chown -R ubuntu:ubuntu /home/ubuntu/yolo

# Rosdep update
RUN rosdep update

# Set environment variables in user bashrc (ROS networking config)
# Source the ROS setup file ---STEP 3
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# ---STEP 7
# Set environment variables in user bashrc (ROS networking config)
#RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo 'source yolo/devel/setup.bash' >> ~/.bashrc

# ros_master_uri = jetson's IP
ARG ROS_MASTER_URI=http://192.168.8.2:11311
# client_ip = my laptops IP
ARG CLIENT_IP=192.168.8.248
ARG DISPLAY_VAR=:0
ENV DISPLAY=${DISPLAY_VAR}

RUN echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc && \
    echo "export ROS_IP=$CLIENT_IP" >>  ~/.bashrc && \
    echo "export DISPLAY=$CLIENT_IP:0"  >> ~/.bashrc


# Copy project
ARG HOME_DIR=/home/$USERNAME/

RUN echo "export PYTHONPATH=$PYTHONPATH:/home/ubuntu/yolo/devel/lib/python3.8/dist-packages" >> /home/ubuntu/.bashrc

# Add user to video group
RUN usermod --append --groups video $USERNAME

# Switch to non-root user
USER $USERNAME

WORKDIR /home/$USERNAME/yolo
