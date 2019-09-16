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
 + Package dependencies
   * First-order dependencies : First Order Dependencies를 확인
    ```bash
    rospack depends1 beginner_tutorials
    ```
   * Indirect dependencies : Recursive하게 모든 Dependencies를 확인
   ```bash
    rospack depends beginner_tutorials
    ```
 + Customizing Package
  * package directory 안의 pacakge.xml 파일을 수정하여 커스터마이징
   - description Tag
   ```
   <description>프로젝트에 대한 대략적인 설명</description>
   ```
   - maintainer Tag : 최소 한명 이상의 package maintainer 있어야 함.
   ```
   <maintainer email="cleverdevk@gmail.com>Inbae Kang</maintainer>
   ```
   - license Tag
   ```
   <license>BSD or MIT or GPLv2....</license>
   ```
   - dependencies Tags : package의 dependency를 명시하는데 4가지로 나뉨. 자세한 내용은 [여기](http://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies)를 참조.
   
     * build_depend : 빌드타임에 패키지 설치에 필요한 패키지들을 명시
     * buildtool_depend : 패키지를 빌드할 때 필요한 build system tool을 명시
     * exec_depend : 패키지를 실행하는데 필요한 패키지들을 명시.
     * test_depend : 오직 unit test를 위해서 필요한 추가적인 패키지를 명시.
   ```
   <!-- Examples -->
   <buildtool_depend>catkin</buildtool_depend>

   <build_depend>message_generation</build_depend>
   <build_depend>roscpp</build_depend>
   <build_depend>std_msgs</build_depend>

   <run_depend>message_runtime</run_depend>
   <run_depend>roscpp</run_depend>
   <run_depend>rospy</run_depend>
   <run_depend>std_msgs</run_depend>

   <test_depend>python-mock</test_depend>
   ```

  
  
 
 
