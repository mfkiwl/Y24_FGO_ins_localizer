# ins_localizer

GNSS/IMU localizer using GTSAM factor graph.

## Install

1. Install [cmake 3.1](https://cmake.org/download/) or higher
    ```
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends libboost-all-dev
    wget https://github.com/Kitware/CMake/releases/download/v3.18.5/cmake-3.18.5-Linux-x86_64.tar.gz
    tar xvf *.tar.gz
    mv cmake-3.18.5-Linux-x86_64 /opt
    ln -f -s /opt/cmake-3.18.5-Linux-x86_64/bin/* /usr/bin
    ln -f -s /opt/cmake-3.18.5-Linux-x86_64/bin/* /usr/local/bin
    ```
1. Install [Eigen 3.4.0](https://github.com/MapIV/eigen.git)
    ```
    git clone https://github.com/MapIV/eigen.git -b 3.4.0
    cd eigen
    mkdir build
    cmake ..
    make -j$(nproc)
    sudo make install
    ```
1. Install [GTSAM 4.2a8](https://github.com/borglab/gtsam/tree/4.2a8)
    ```
    git clone https://github.com/borglab/gtsam.git -b 4.2a8
    cd gtsam
    mkdir build
    cd build
    cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON
    make -j$(nproc)
    sudo make install
    ```
1. Install [iridescence](https://github.com/koide3/iridescence)
    ```
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends lld libpng-dev libjpeg-dev libglm-dev libglfw3-dev libfmt-dev libspdlog-dev libassimp-dev zenity python3-pip
    apt-get clean
    rm -rf /var/lib/apt/lists/*
    git clone https://github.com/koide3/iridescence --recursive
    mkdir iridescence/build
    cd iridescence/build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
    ```
1. Install this repository
    ```
    git clone git@github.com:MeijoMeguroLab/Y24_FGO_ins_localizer.git
    cd Y24_FGO_ins_localizer
    mkdir build
    cd build
    apt-get update
    apt-get install -y --no-install-recommends libyaml-cpp-dev libpcl-dev
    cmake ..
    make -j$(nproc)
    ```

### Docker

#### Build Image

```
docker build -t ins_localizer -f docker/Dockerfile
```

#### Run Container

```
docker run -it --rm --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ins_localizer:latest
```

## Run with sample data

```
cd ins_localizer/build
./offline_ins_localizer ../sample_data/GNSS_enu_data_25Hz.csv ../sample_data/imu_scha_data.csv ../sample_data/velocity_data.csv ../sample_data/eagleye_rpy_data.csv ../config/sample.yaml ../sample_data/output.csv
```
