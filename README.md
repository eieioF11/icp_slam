# icp_slam

## environment
- ubuntu22.04
- ros2 humble

## test
- LRF : YDLiDAR x4
- IMU : wt901c rs232 ver

## Install
### ydlidar sdk install
```bash
sudo apt install cmake pkg-config
```
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

### ydlidar usb setup
```bash
cd YDLidar-SDK/startup
sudo sh initenv.sh
```

### ydlidar ros2 pkg
```bash
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ydlidar_ros2_driver/
git checkout humble
```

### wt901c install
```bash
sudo apt install ros-humble-sophus
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/stonier/ecl_core -b devel
git clone https://github.com/stonier/ecl_tools -b devel
git clone https://github.com/stonier/ecl_lite -b devel

git clone https://github.com/fateshelled/wit_node -b ros2

cd ~/ros2_ws
colcon build --symlink-install
```
### wt901c test
```
ros2 launch wit_node wit_visualize.launch.py

# ROS2ノードのボーレートを変更する場合
ros2 launch wit_node wit_visualize.launch.py　baud_rate:=9600
```
