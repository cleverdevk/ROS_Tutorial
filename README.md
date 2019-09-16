# ROS_Tutorial

1. Installation
---------------

* Set up your source.list
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
* Set up Keys
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
* Update debian package index
```bash
sudo apt-get update
```
* ROS Install
```bash
sudo apt-get install ros-kinetic-desktop-full
```
* Initialize ROSDEP
```bash
sudo rosdep init
rosdep update
```
* Envirnment Setup
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
* Istall dependencies Packages
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
* Create ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
2. ROS File System
-------------------

* 진행하기 위해서는 ROS Tutorial 패키지를 설치해야 함. (distro는 버전에 따라 다름. 여기선 kinetic)
```bash
sudo apt-get install ros-<distro>-ros-tutorials
```
* File System Tools
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

* Catkin package는 적어도 CMakeList.txt 와 package.xml 파일을 가지고 있어야 함.

* catkin package 만들기
 * 디렉토리 이동 후 패키지 생성(dependencies 포함)
 ```bash
 cd ~/catkin_ws/src
 
 catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
 ```
 * 일반적인 Form
 ```bash
 catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
 ```
* Building a catkin workspace and sourcing the setup file
 * 디렉토리 이동 후 catkin_make
 ```bash
 cd ~/catkin_ws
 catkin_make
 source ~/catkin_ws/devel/setup.bash
 ```
 * Package dependencies
  * First-order dependencies : First Order Dependencies를 확인
  ```bash
  rospack depends1 beginner_tutorials
  ```
  * Indirect dependencies : Recursive하게 모든 Dependencies를 확인
  ```bash
  rospack depends beginner_tutorials
  ```
 * Customizing Package
  * package directory 안의 pacakge.xml 파일을 수정하여 커스터마이징
   * description Tag
   ```
   <description>프로젝트에 대한 대략적인 설명</description>
   ```
   * maintainer Tag : 최소 한명 이상의 package maintainer 있어야 함.
   ```
   <maintainer email="cleverdevk@gmail.com>Inbae Kang</maintainer>
   ```
   * license Tag
   ```
   <license>BSD or MIT or GPLv2....</license>
   ```
   * dependencies Tags : package의 dependency를 명시하는데 4가지로 나뉨. 자세한 내용은 [여기](http://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies)를 참조.
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

4. Building a ROS Package
-----------------------

* Building Packages
 * catkin_make 사용하기 : catkin_make는 cmake와 make를 결합한 call이라고 생각하면 됨.
 ```bash
 # catkin workspace에서..(catkin_ws)
 catkin_make
 catkin_make install # (optionally)
 ```
 위의 명령어는 아래의 CMake 프로젝트의 Flow와 비슷한 흐름이라고 볼 수 있다.
  
 ```bash
 # In a CMake project
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make
 $ make install  # (optionally)
 ```
5. Understanding ROS Nodes
-----------------------------

* Overview of Graph Concepts
 * [Nodes](http://wiki.ros.org/Nodes) : An executable that uses ROS to communicate with other nodes.
  * 계산을 수행하는 프로세스. 노드들은 그래프에 결합되고 다른 노드들과 통신한다.
  * 통신에는 [Streaming Topics](#streaming_topics), [RPC Services](#rpcservices), [Parameter Server](#parameterserver)를 사용한다.
  * 실행중인 모든 노드는 나머지 시스템 노드들과 고유하게 식별될 수 있는 [Graph Resource Name](http://wiki.ros.org/Names)이 있다.
  * 또한 노드는 Node Type을 가지는데, 노드의 패키지 이름과 노드의 실행파일 이름 그리고 [Package Resource Name](http://wiki.ros.org/Names)이다.
  * ROS는 이름을 통해서 모든 executable을 찾고 가장 먼저 찾아진 것을 고르기 때문에, 같은 이름의 다른 실행파일을 만들지 않도록 주의해야 한다.
  * Command-line Remapping Arguments
   ROS의 강력한 기능중 하나로, 같은 노드를 많은 환경설정으로 실행할 수 있다.
   ```bash
   rosrun rospy_tutorials talker chatter:=/wg/chatter
   ```
   이렇게 실행을 하게 되면, talker에게 chatter대신 /wg/chatter에게 publish하도록 Remapping 할 수 있다.
   Remapping되는 예시는 다음과 같다.
   
| Node Namespace | Remapping Argument | Matching Names | Final Resolved Name |
|:---------------|:-------------------|:---------------|:--------------------|
| /              | foo:=bar           | foo, /foo      | /bar                |
| /baz           | foo:=bar           | foo, /baz/foo  | /baz/bar            |
| /              | /foo:=bar          | foo, /foo      | /bar                |
| /baz           | /foo:=bar          | /foo           | /baz/bar            |
| /baz           | /foo:=/a/b/c/bar   | /foo           | /a/b/c/bar          |
  
  * 그 외의 자세한 내용들은 [여기](http://wiki.ros.org/Nodes)를 참조.
  
 * [Messages](http://wiki.ros.org/Messages) : ROS data type used when subscribing or publishing to a topic.
  * Node들은 message를 topic에 publish해서 각 노드들끼리 통신한다.
  * message는 지정된 field들로 구성된 간단한 data structure이다.
  * Standard primitive types(integer, floating point, boolean, etc..)가 지원되며, 구조체와 같은 primitive나 배열도 포함될 수 있다.
  * 또한 ROS Service Call을 통해서 Request와 Response를 주고받을 수 있는데, 이러한 Request와 Response Message는 [srv file](http://wiki.ros.org/srv)로 정의됨.
   * [msg](http://wiki.ros.org/msg) files : message의 data structure를 명시하기 위한 simple text file. package의 msg라는 subdirectory에 저장된다. 또한 노드는 메시지 유형과 MD5 sum이 일치해야만 통신할 수 있다.
   * Message Types : Standard ROS [naming](http://wiki.ros.org/Names)을 사용하는데, "패키지이름/메시지파일이름" 이러한 형식이다.
   * Building : CMakeList.txt파일 안에서 rosbuild_genmsg()를 통해서 가능.
   * 그 외의 자세한 내용들은 [여기](http://wiki.ros.org/Messages)를 참조.
   
