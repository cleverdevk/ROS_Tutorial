# ROS_Tutorial
ROS
ROS Tutorial
============

1. Installation
---------------

+ 1) Set up your source.list
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
+ 2) Set up Keys
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
+ 3) Update debian package index
```bash
sudo apt-get update
```
+ 4) ROS Install
```bash
sudo apt-get install ros-kinetic-desktop-full
```
+ 5) Initialize ROSDEP
```bash
sudo rosdep init
rosdep update
```
+ 6) Envirnment Setup
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
+ 7) Istall dependencies Packages
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
+ 8) Create ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
2. ROS File System
-------------------

+ 1) 진행하기 위해서는 ROS Tutorial 패키지를 설치해야 함. (distro는 버전에 따라 다름. 여기선 kinetic)
```bash
sudo apt-get install ros-<distro>-ros-tutorials
```
+ 2) File System Tools
  * rospack : package에 대한 정보를 얻을 수 있음.
   * rospack find : package 위치 정보 확인
    ```bash
     rospack find roscpp
     ```
  * roscd
   * roscd : ros package의 위치로 이동.(서브디렉토리로도 이동 가능)
   ```bash
     roscd roscpp
     roscd roscpp/cmake
     ```
    * roscd log : ROS가 log를 저장하는 디렉토리로 이동.
    ```bash
     roscd log
     ```
   * rosls : ROS 디렉토리들의 파일 목록 바로 조회.
   ```bash
    rosls roscpp_tutorial
    ```
    
    * TAB 자동완성 가능.
