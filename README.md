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

  * [Topics](http://wiki.ros.org/Topics) : Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
     * 노드들이 메시지를 교환하는 **이름이 있는** 버스라고 생각하면 됨.
     * 정보의 production과 consumption을 분리하여 익명의 publish/subscribe를 가짐.
     * 따라서 노드들은 각각 누구와 통신하는지 모름. 대신에 관심이 있는 정보에 대해서 **Subscribe**하거나 **Publish**하는 것.
     * 이러한 특성에 따라서 토픽에 다수의 publisher/subscriber가 존재할 수 있음.
     * Topics은 그래서 undicrectional하고, Streaming Communication이다. 
     * Remote Procedure Call을 수행해야 하는 Node는(즉 요청에 응답해야하는 노드) 대신 Services를 사용해야 함.
     * 적은 양의 상태를 유지하는 [Parameter Server](http://wiki.ros.org/Parameter%20Server)가 존재함.
     * 각각의 Topic은 해당 Topic에 publish하는데 사용되는 ROS Message Type에 의해서 type이 지정되며 각 노드는 일치하는 type의 메시지만 받을 수 있음. 이 과정에서, 모든 ROS Client는 MD5를 계산하고 일치하는지도 확인하여 일관된 코드 베이스에서 컴파일 되었는지도 체크함.
     * Master 노드(아래 참조)는 type consistency에 제약을 받지 않는데, subscriber는 type이 매치되기 전까지는 message 전송을 하지 않음.
     * TCP/IP기반으로 전송함. UDP는 현재 roscpp만 지원하니까 패스함.
     * 그 외의 자세한 내용들은 [여기](http://wiki.ros.org/Topics)를 참조.

  * [Master](http://wiki.ros.org/Master) : Name service for ROS (i.e. helps nodes find each other)
     * ROS Master는 ROS system의 node들의 naming과 registration을 할 수 있도록 한다.
     * Topic과 Service에 대한 Publisher/Subscriber를 추적함.
     * Master의 역할은 ROS의 개별 노드들이 서로를 찾을 수 있도록 하는 것임.
     * 이러한 노드들이 서로를 찾으면 서로 peer-to-peer communication을 함.
     * Master가 Parameter Server를 제공함.
     * roscore 명령어로 다른 필수적인 components와 함께 실행됨.
     * naming 및 그외의 자세한 내용들은 [여기](http://wiki.ros.org/Master)를 참조.
     
  * [rosout](http://wiki.ros.org/rosout) : ROS equivalent of stdout/stderr
  * [roscore](http://wiki.ros.org/roscore) : Master + rosout + parameter server (parameter server will be introduced later)
  

* Node들은 Service를 사용하거나 제공할 수 있다.

* rosrun을 통해서 노드 생성하기
  1) roscore로 master 노드 생성 및 ROS 실행
  ```bash
  roscore
  ```
  ![roscore](https://i.imgur.com/EevvwJ0.png)
  
  2) 새로운 터미널을 열고 rosrun으로 거북이 실행하기
  * rosrun '패키지이름' '노드이름' 형식.
  ```bash
  rosrun turtlesim turtlesim_node
  ```
  ![rosrun](https://i.imgur.com/e1ybWqR.png)
  
  
  3) 새로운 터미널을 열고 노드이름을 다르게 거북이 실행하기
  ```bash
  rosrun turtlesim turtlesim_node __name:=my_turtle
  ```
  4) 새로운 터미널을 열고 현재 노드 확인하기.
  ```bash
  rosnode list
  ```
  ![rosnode list](https://i.imgur.com/CXtNPeB.png)
  * 두 개의 노드가 추가된 것을 알 수 있다.
  
  
6. Understanding ROS Topics
-----------------------------

* Turtlesim Teleoperation : 거북이를 키보드 방향키로 움직이도록 할 수 있는 패키지
```bash
rosrun turtlesim turtle_teleop_key
```
를 실행하고 그 터미널에서 방향키를 누르면 거북이가 움직인다.
왜냐하면 turtle_teleop_key가 turtle1/cmd_vel이라는 Topic에 Publish하고 turtlesim_node가 Subscribe하기 때문에 가능하다.
현재 Node 및 Topic에 관한 그래프는 rqt_graph로 확인 가능하다.
```bash
#설치가 안되어 있다면
sudo apt-get install ros-kinetic-rqt
sudo apt-get install ros-kinetic-rqt-common-plugins

#그래프 확인은 새로운 터미널을 열고
rosrun rqt_graph rqt_graph
```
![rqt](https://i.imgur.com/qG07xfc.png)

- rostopic 명령어 : rostopic tool은 ROS topic에 대한 정보들을 얻을 수 있는 명렁어
    - rostopic -h : help option
    - rostopic echo : 해당 토픽에 publish되는 데이터를 볼 수 있다.
    ```bash
    rostopic echo /turtle1/cmd_vel
    ````
    rostopic echo를 켠 상태로 거북이를 움직이면, 메시지가 기록된다.

    ![echo](https://i.imgur.com/Nw1a6ec.png)

- ROS Messages
    - rostopic type [topic] : 토픽의 type 확인
    - rosmsg show [type of topic] : message의 detail 확인
    ```bash
    rostopic type /turtle1/cmd_vel
    rosmsg show geometry_msgs/Twist
    ```
    ![terminal](https://i.imgur.com/j0iNEVu.png)

    - rostopic pub [topic] [msg_type] [args] : currently advertised topic에 data를 publish할 수 있음.
    ```bash
    rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    ```
    - -1 option : 오직 한개의 message만 publish하고 exit
    - — : 다음 인자가 옵션이 아님을 알려주는 것.
    - 두개의 coordination은 linear와 angular에 대한 인자이다.

    그 외의 자세한 내용은 [여기](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)를 참조.

7. Understanding ROS Services and Parameters
-----------------------------

* ROS Service : Node들이 서로 통신할 수 있는 또 다른 방법. Services는 request를 보내고 response를 받을 수 있도록 해줌.
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```
