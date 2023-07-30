# seek_compact_pro_node
A ROS node to expose the SEEK Compact Pro heat camera


## Install

#### OpenCV3
`opencv-3.4.9`

```
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
 libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
 libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
 gfortran openexr libatlas-base-dev python3-dev python3-numpy \
 libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
 libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
mkdir ~/opencv_build && cd ~/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd ~/opencv_build/opencv
mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
 - D CMAKE_INSTALL_PREFIX=/usr/local \
 - D INSTALL_C_EXAMPLES=ON \
 - D INSTALL_PYTHON_EXAMPLES=ON \
 - D OPENCV_GENERATE_PKGCONFIG=ON \
 - D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
 - D BUILD_EXAMPLES=ON ..
 make -j8
sudo checkinstall
```

#### git clone and  build

```
cd ~/ros/src/
git clone --recursive git@github.com:AiriYokochi/seek_compact_pro_node.git
cd seek_compact_pro_node/src/libseek-thermal
mkdir build
cd build
cmake ..
sudo checkinstall
sudo ldconfig 
cd ../../
catkin bt
source /opt/ros/noetic/setup.bash
```

## launch
```
roslaunch seek_compact_pro_node seek_compact_pro_node.launch
```
