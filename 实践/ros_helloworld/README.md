# ros_helloworld

ros_helloworld软件包，包括C++与Python两个版本的helloworld信的示例。

## 功能介绍

在命令行显示一句话："ROS Hello World!!!! by cpp"

C++版本代码见`src/`下的[ros_helloworld_c.cpp](./src/ros_helloworld_c.cpp)。

Python版本代码见`scripts/`下的[ros_helloworld_p.py](./scripts/ros_helloworld_p.py)。


## 运行方法

启动节点

```sh
$ rosrun ros_helloworld ros_helloworld_p.py   #Python
$ rosrun ros_helloworld ros_helloworld_c_node        #C++
``` 

启动launch文件

```sh
$ roslaunch ros_helloworld start_turtle.launch  #launch
``` 
