# topic_demo

topic_demo软件包，包括C++与Python两个版本的Topic通信的示例。

## 功能介绍

假设Topic的发布者为GPS模块，它以**1HZ**的频率向**/gps_info**这个topic上发布消息，消息格式要包括坐标(x,y)和工作状态(state)。

Topic的接受者会订阅**/gps_info**，并计算每次GPS位置到原点的距离，在屏幕上显示。

本例需要自定义msg文件，见[msg/Gps.msg](./msg/Gps.msg)。

C++版本代码见`src/`下的[talker.cpp](./src/talker.cpp)和[listener.cpp](./src/listener.cpp)。

Python版本代码见`scripts/`下的[pytalker.py](./scripts/pytalker.py)和[pylistener.py](./scripts/pylistener.py)。


## 运行方法

启动发布者
```sh
$ rosrun topic_demo pytalker.py   #Python
$ rosrun topic_demo talker        #C++
``` 

启动接收者
```sh
$ rosrun topic_demo pylistener.py   #Python
$ rosrun topic_demo listener        #C++
``` 

## 三种运行python节点的方式都可行
$ rosrun topic_demo pytalker.py
$ rosrun topic_demo pylistener.py
或者
$ cd /home/zqc/catkin_ws/src/ros_study/topic_demo/scripts/
$ ./pytalker.py
$ ./pylistener.py
或者
$ cd /home/zqc/catkin_ws/src/ros_study/topic_demo/scripts/
$ python pytalker.py.py
$ python pylistener.py

但是python pytalker.py这种方法是最方便的，不需要设置为可执行文件，不需要CMakeLists.txt设置catkin_install_python()，只要是python xxx.py都可以运行
