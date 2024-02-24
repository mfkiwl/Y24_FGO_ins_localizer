# ARG VERSION=22.02
# ARG PYTHON=3
# 
# FROM nvcr.io/nvidia/tensorrt:${VERSION}-py${PYTHON}
FROM ubuntu:jammy

ENV DEBIAN_FRONTEND noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    wget \
    build-essential \
    ca-certificates \
    locales \
    git \
    lsb-release && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libboost-all-dev && \
    wget https://github.com/Kitware/CMake/releases/download/v3.18.5/cmake-3.18.5-Linux-x86_64.tar.gz && \
    tar xvf *.tar.gz && \
    mv cmake-3.18.5-Linux-x86_64 /opt && \
    ln -f -s /opt/cmake-3.18.5-Linux-x86_64/bin/* /usr/bin && \
    # This specific version of cmake needs to be copied to /usr/local/bin as well. 
    # Otherwise, gpu-base will break because it has an older version of cmake there
    ln -f -s /opt/cmake-3.18.5-Linux-x86_64/bin/* /usr/local/bin && \
    rm cmake-3.18.5-Linux-x86_64.tar.gz
    # Install Eigen
RUN git clone https://github.com/MapIV/eigen.git -b 3.4.0 && \
    cd eigen && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../../ && \
    rm -rf eigen
    # Install GTSAM
RUN git clone https://github.com/borglab/gtsam.git -b 4.2a8 && \
    cd gtsam && \
    mkdir build && \
    cd build && \
    cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j$(nproc) && \
    make install && \
    cd ../../ && \
    rm -rf gtsam
    # Install Iridescence
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    lld libpng-dev libjpeg-dev libglm-dev libglfw3-dev \
    libfmt-dev libspdlog-dev libassimp-dev zenity python3-pip && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    git clone https://github.com/koide3/iridescence --recursive && \
    mkdir iridescence/build && cd iridescence/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    rm -rf iridescence

COPY ./ /root/ins_localizer/
RUN cd /root/ins_localizer/ && \
    mkdir build && \
    cd build && \
    apt-get update && \
    apt-get install -y --no-install-recommends libyaml-cpp-dev libpcl-dev && \
    cmake .. && \
    make -j$(nproc)

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

CMD ["bash"]
