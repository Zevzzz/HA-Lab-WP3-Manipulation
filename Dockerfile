FROM osrf/ros:jazzy-desktop-full
MAINTAINER Donson Xie

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN userdel -r ubuntu || true

RUN apt-get update && apt-get install -y \
    vim net-tools locales iputils-ping curl \
    software-properties-common git \
    python3-pip python3-venv \
    python3-colcon-common-extensions python3-rosdep \
    mesa-utils x11-apps \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Python 3.10 for GraspGen
#RUN add-apt-repository -y ppa:deadsnakes/ppa \
#    && apt-get update \
#    && apt-get install -y python3.10 python3.10-venv python3.10-dev \
#    && rm -rf /var/lib/apt/lists/*
#    
#RUN apt-get update && apt-get install -y wget \
#    && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-#keyring_1.1-1_all.deb \
#    && dpkg -i cuda-keyring_1.1-1_all.deb \
#    && rm -f cuda-keyring_1.1-1_all.deb \
#    && apt-get update \
#    && apt-get install -y cuda-toolkit-12-1 \
#    && rm -rf /var/lib/apt/lists/*
#ENV CUDA_HOME=/usr/local/cuda
#ENV PATH="${CUDA_HOME}/bin:${PATH}"
#ENV LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"

# ROS deps for sim + planning
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

ARG USER_ID=1000
ARG GROUP_ID=1000
ENV USER=ros
ENV HOME=/home/${USER}

RUN groupadd --gid ${GROUP_ID} ${USER} \
    && useradd --uid ${USER_ID} --gid ${GROUP_ID} --shell /bin/bash --create-home ${USER} \
    && adduser ${USER} sudo \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV WS=${HOME}/ws
WORKDIR ${WS}
USER ${USER}

# Copy local src in (optional — compose will also mount it)
COPY --chown=${USER}:${USER} ./ws/src ./src

RUN sudo chown -R ${USER}:${USER} ${WS} \
    && source /opt/ros/jazzy/setup.bash \
    && sudo rosdep init || true \
    && rosdep update \
    && if [ -d "src" ] && [ "$(ls -A src 2>/dev/null)" ]; then \
         rosdep install --from-paths src --ignore-src -r -y --rosdistro=jazzy; \
         colcon build --symlink-install; \
       else \
         echo "No src packages to build (ok)."; \
       fi

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

CMD ["bash"]
