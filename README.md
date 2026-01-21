# Project Setup and Installation Guide

This guide details the steps to set up the ROS 2 Humble environment, configure the Python virtual environment, install dependencies, and build `stella_vslam` with RealSense support.

## 1. System Setup & ROS 2 Humble Installation

First, ensure the locale is set correctly and install ROS 2 Humble.

```bash
# Set Locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 Repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Base and Dev Tools
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# Install Additional ROS Packages (Nav2, Vision, Gazebo, Turtlebot3)
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-vision-opencv \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3* \
    python3-pip python3-venv
```

## 2. Python Environment Setup

We use a virtual environment with access to system site packages to manage Python dependencies without breaking the system ROS installation.

```bash
# Create the environment with access to system packages
python3 -m venv --system-site-packages ~/ros2_env

# Activate the environment (Required before building/running)
source ~/ros2_env/bin/activate

# Upgrade pip inside the env
pip install --upgrade pip

# Note: DO NOT install rclpy or opencv via pip here unless strictly necessary.
pip install ultralytics deep-sort-realtime
pip install "numpy<2.0" scipy
pip install setuptools==58.2.0  # Downgrade setuptools to avoid warnings in ROS 2 builds
pip install transforms3d
pip install cffi

# Configure .bashrc for auto-setup (Optional)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_env/bin/activate" >> ~/.bashrc 
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:/home/samuele/ros2_env/lib/python3.10/site-packages" >> ~/.bashrc
source ~/.bashrc
```

## 3. Workspace Initialization

Create the workspace and install ROS dependencies.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone this repo 

# Install dependencies defined in package.xml files
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

## 4. Installing Stella VSLAM Dependencies

This section covers the installation of system libraries and manual compilation of required third-party libraries.

### 4.1 System Dependencies

```bash
sudo apt update -y
sudo apt upgrade -y --no-install-recommends

# Basic tools
sudo apt install -y build-essential pkg-config cmake git wget curl unzip

# Math & Linear Algebra
sudo apt install -y libatlas-base-dev libsuitesparse-dev gfortran

# OpenCV System Dependencies
sudo apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libtbb-dev

# Other Utils
sudo apt install -y binutils-dev libyaml-cpp-dev libgflags-dev sqlite3 libsqlite3-dev libglew-dev

# Create the common library directory
mkdir -p ~/lib
```

### 4.2 Manual Build of Libraries

**Eigen 3.3.7**
```bash
cd ~/lib
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
tar xf eigen-3.3.7.tar.bz2 && rm -rf eigen-3.3.7.tar.bz2
cd eigen-3.3.7
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4 && sudo make install
```

**OpenCV 4.5.5 (Manual Build)**
```bash
cd ~/lib
# Download OpenCV Base
wget -q https://github.com/opencv/opencv/archive/4.5.5.zip
unzip -q 4.5.5.zip && rm 4.5.5.zip

# Download OpenCV Contrib (Extra Modules)
wget -q https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip -O opencv_contrib-4.5.5.zip
unzip -q opencv_contrib-4.5.5.zip && rm opencv_contrib-4.5.5.zip

# Build OpenCV
cd ~/lib/opencv-4.5.5
mkdir build && cd build

cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DENABLE_CXX11=ON \
    -DENABLE_FAST_MATH=ON \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_TBB=ON \
    -DWITH_OPENMP=ON \
    -DOPENCV_EXTRA_MODULES_PATH=~/lib/opencv_contrib-4.5.5/modules \
    ..
make -j4
sudo make install
sudo ldconfig
```

**FBoW (stella-cv version)**
```bash
cd ~/lib
git clone https://github.com/stella-cv/fbow.git
cd fbow
mkdir build && cd build
cmake ..
make -j$(nproc) && sudo make install
sudo ldconfig
```

**g2o (Graph Optimization)**
```bash
cd ~/lib
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o && git checkout 20230223_git
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
    ..
make -j4 && sudo make install
```

**backward-cpp**
```bash
cd ~/lib
git clone https://github.com/bombela/backward-cpp.git
cd backward-cpp && git checkout 5ffb2c879ebdbea3bdb8477c671e32b1c984beaa
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4 && sudo make install
```

**Pangolin**
```bash
cd ~/lib
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && git checkout eab3d3449a33a042b1ee7225e1b8b593b1b21e3e
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_EXAMPLES=OFF ..
make -j4 && sudo make install
```

## 5. Building Stella VSLAM

```bash
# 1. Install Core Library
# 1. Install Core Library
cd ~/lib
git clone --recursive https://github.com/stella-cv/stella_vslam.git
cd stella_vslam && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j4 && sudo make install

# 2. Install Pangolin Viewer
cd ~/lib
git clone --recursive https://github.com/stella-cv/pangolin_viewer.git
mkdir -p pangolin_viewer/build && cd pangolin_viewer/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j && sudo make install

# 3. Install Examples (Optional)
cd ~/lib
git clone --recursive https://github.com/stella-cv/stella_vslam_examples.git
mkdir -p stella_vslam_examples/build && cd stella_vslam_examples/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_STACK_TRACE_LOGGER=ON ..
make -j

# 4. Build ROS 2 Package
cd ~/lib/stella_vslam/build
source /opt/ros/${ROS_DISTRO}/setup.bash
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j && sudo make install

# 5. Build Workspace
cd ~/ros2_ws
rosdep install -y -i --from-paths ~/ros2_ws/src --skip-keys=stella_vslam
colcon build --symlink-install
```

## 6. RealSense Setup

Install drivers and ROS 2 wrapper for Intel RealSense cameras.

```bash
# Register Public Key and Repo
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

# Install SDK and ROS Pkg
sudo apt update
sudo apt install librealsense2-utils librealsense2-dkms
sudo apt install ros-humble-realsense2-camera
```

## 7. Troubleshooting & Final Configuration

### OpenCV Version Conflict Fix
If you encounter linking errors due to conflicts between the manual OpenCV install and the system ROS OpenCV, follow these steps to remove the manual version and link against the system version.

```bash
# 1. Remove manually installed OpenCV
sudo rm -rf /usr/local/lib/libopencv*
sudo rm -rf /usr/local/include/opencv4
sudo rm -rf /usr/local/lib/cmake/opencv4

# 2. Rebuild dependencies (g2o, Pangolin, fbow) to link against system libraries
cd ~/lib/g2o/build
rm -rf * && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

cd ~/lib/Pangolin/build
rm -rf * && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

cd ~/lib/fbow/build
rm -rf * && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# 3. Rebuild Stella VSLAM
cd ~/lib/stella_vslam/build
rm -rf * && cmake .. 
# Verify output says: "Found OpenCV: /usr (found version 4.5.4)"
make -j$(nproc) && sudo make install && sudo ldconfig

# 4. Verify Linkage
ldd /usr/local/lib/libstella_vslam.so | grep opencv_core
# Correct output should point to: /lib/x86_64-linux-gnu/...

# 5. Final Workspace Clean & Build
cd ~/ros2_ws
rm -rf build install
colcon build --symlink-install --cmake-args -Dstella_vslam_DIR=/usr/local/lib/cmake/stella_vslam
source install/setup.bash
```

### Fix Window Crashes
Install legacy GTK libs if the camera viewer crashes.

```bash
sudo apt update
sudo apt install -y libgtk2.0-dev pkg-config libcanberra-gtk-module
```