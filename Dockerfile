FROM ros:humble-ros-base

# config apt
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    apt update && apt upgrade -y && \
    apt-get install ros-humble-foxglove-bridge wget htop vim libceres-dev libpcl-ros-dev ros-humble-xacro -y


# install openvino, but we should not install it when in arm32/64
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2023.list && \
    apt-get update && apt install openvino-2023.2.0 -y && \
    rm -rf /var/lib/apt/lists/* 

# create workspace
RUN mkdir -p /autoaim_ws
WORKDIR /autoaim_ws/
# Copy Project
COPY . /autoaim_ws/src

# install dependencies and some tools
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y 

# build
RUN  touch src/helios_autoaim/autoaim_fire_controller/COLCON_IGNORE && \
     . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

ARG ROS_DOMAIN_ID
# setup .zshrc
RUN echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}\n\
          source /autoaim_ws/install/setup.sh' \
          >> /root/.bashrc

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/autoaim_ws/install/setup.sh"' \
      /ros_entrypoint.sh