# Base image with ROS noetic (Python3.8 default)
FROM osrf/ros:noetic-desktop

# Create non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN rm -f /etc/apt/sources.list.d/ros1-latest.list \
  && rm -f /usr/share/keyrings/ros1-latest-archive-keyring.gpg

# ─── Python tooling & OpenCV ───────────────────────────────────────────────
RUN sudo apt-get update && sudo apt-get install -y \
      python3-pip \
    && pip3 install --no-cache-dir opencv-python \
    && pip3 install --no-cache-dir kaggle gdown tqdm

# ─── PyTorch 1.8.1 CPU-only + TorchVision/Torchaudio ───────────────────────
RUN pip3 install --no-cache-dir \
      torch==1.8.1+cpu \
      torchvision==0.9.1+cpu \
      torchaudio==0.8.1 \
      -f https://download.pytorch.org/whl/torch_stable.html

# ─── YOLOv5 (latest) + its requirements ────────────────────────────────────
#RUN git clone --depth 1 https://github.com/ultralytics/yolov5.git /home/$USERNAME/yolov5 \
#RUN pip3 install --no-cache-dir -r /home/$USERNAME/yolov5/requirements.txt \
# && pip3 install --no-cache-dir ultralytics

# jetson_camera, motion requirements
COPY yolo/requirements.txt /requirements.txt
# yolov8
RUN pip3 install --no-cache-dir ultralytics
# jetson_camera, motion requirements
RUN pip3 install --no-cache-dir -r /requirements.txt

COPY yolo /home/ubuntu/yolo

# Change ownership ---STEP 1
RUN chown -R ubuntu:ubuntu /home/ubuntu/yolo

COPY yolo/startup.sh /home/ubuntu/yolo/startup.sh
RUN chmod +x /home/ubuntu/yolo/startup.sh && chown ubuntu:ubuntu /home/ubuntu/yolo/startup.sh


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
