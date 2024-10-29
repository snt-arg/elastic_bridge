FROM nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    wget \
    curl \
    cmake-qt-gui \
    libusb-1.0-0-dev \
    libudev-dev \
    openjdk-11-jdk \
    freeglut3-dev \
    libglew-dev \
    libsuitesparse-dev \
    zlib1g-dev \
    libjpeg-dev \
    libssl-dev \
    libepoxy-dev

# upgrade cmake to 3.22
ARG version=3.22
ARG build=1
RUN apt remove cmake -y
WORKDIR /tmp
RUN wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
RUN tar -xzvf cmake-$version.$build.tar.gz
WORKDIR /tmp/cmake-$version.$build
RUN ./bootstrap
RUN make -j8
RUN make install

# install ROS Noetic
ENV ROS_DISTRO=noetic

# setup keys
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# ROS related packages 
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-vcstool \
    ros-noetic-pcl-ros \
    ros-noetic-backward-ros \
    ros-noetic-rviz-visual-tools \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf \
    ros-noetic-pcl-ros \
    ros-noetic-tf-conversions

# ROS dependencies
RUN rosdep init
RUN rosdep update

# Install Rust rustup toolchain for mprocs
WORKDIR /workspace
RUN curl https://sh.rustup.rs -sSf | sh -s -- --default-toolchain stable -y
ENV PATH=/root/.cargo/bin:$PATH
RUN cargo install mprocs

# Install ElasticFusion and dependencies
WORKDIR /opt
RUN git clone https://github.com/mp3guy/ElasticFusion.git
WORKDIR /opt/ElasticFusion
RUN git checkout 7fca730813c6ac8973cdddcc26c7538d2dee5074
RUN git submodule update --init
WORKDIR /opt/ElasticFusion/third-party/OpenNI2
RUN make -j8
WORKDIR /opt/ElasticFusion/third-party/Pangolin
RUN mkdir build
WORKDIR /opt/ElasticFusion/third-party/Pangolin/build
RUN cmake .. -DEIGEN_INCLUDE_DIR=/opt/ElasticFusion/third-party/Eigen/ -DBUILD_PANGOLIN_PYTHON=false
RUN make -j8
WORKDIR /opt/ElasticFusion
RUN mkdir build
WORKDIR /opt/ElasticFusion/build
# [Note] - compute capability for GPUs V100 and T600 respectively - can add or remove as needed
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_ARCH_BIN="70 75" 
RUN make -j8

# Install ElasticBridge
RUN mkdir -p /workspace/src
WORKDIR /workspace/src
RUN git clone https://github.com/snt-arg/elastic_bridge.git
RUN git clone https://github.com/RMonica/init_fake_opengl_context.git
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build elastic_bridge -j8"

