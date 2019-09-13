# ROS_Tutorial
ROS
ROS Tutorial
============

1. Installation
---------------

+ Set up your source.list
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
+ Set up Keys
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
+ Update debian package index
```bash
sudo apt-get update
```
+ ROS Install
```bash
sudo apt-get install ros-kinetic-desktop-full
```
+ Initialize ROSDEP
```bash
sudo rosdep init
rosdep update
```
+ Envirnment Setup
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
+ Istall dependencies Packages
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
+ Create ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
2. ROS File System
-------------------

+ 진행하기 위해서는 ROS Tutorial 패키지를 설치해야 함. (distro는 버전에 따라 다름. 여기선 kinetic)
```bash
sudo apt-get install ros-<distro>-ros-tutorials
```
+ File System Tools
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

3. Creating ROS Package
-----------------------

+ Catkin package는 적어도 CMakeList.txt 와 package.xml 파일을 가지고 있어야 함.

+ catkin package 만들기
 * 디렉토리 이동 후 패키지 생성(dependencies 포함)
 ```bash
 cd ~/catkin_ws/src
 
 catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
 ```
  - 일반적인 Form
  ```bash
  catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
  ```
+ Building a catkin workspace and sourcing the setup file
 * 디렉토리 이동 후 catkin_make
 ```bash
 cd ~/catkin_ws
 catkin_make
 source ~/catkin_ws/devel/setup.bash
 ```
 
 
 
