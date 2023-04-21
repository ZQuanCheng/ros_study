# service_demo

Service通信示例，用C++与Python两个版本实现service通信。

## 功能介绍

假设Service的提供方/服务器端提供名为`greetings`的服务，其srv的请求是姓名和年龄，反馈为一个字符串，向请求方问好。

本例需要自定义srv文件，见[srv/Greeting.srv](./srv/Greeting.srv)。

C++版本代码见`src/`下的[client.cpp](./src/client.cpp)和[server.cpp](./src/server.cpp)。

Python版本代码见`scripts/`下的[pyclient.py](./scripts/pyclient.py)和[pyserver.py](./scripts/pyserver.py)。

## 运行方法

启动服务器端
```sh
$ rosrun service_demo pyserver.py   #Python
$ rosrun service_demo server           #C++
``` 

启动客户端
```sh
$ rosrun service_demo pyclient.py   #Python
$ rosrun service_demo client           #C++
``` 

## 修改为可从命令行读取参数的形式

需要自定义srv文件，见[srv/AddTwoInts.srv](./srv/AddTwoInts.srv)。

Python版本代码见`scripts/`下的[pyclient_add.py](./scripts/pyclient_add.py)和[pyserver_add.py](./scripts/pyserver_add.py)。

启动服务器端
```sh
$ rosrun service_demo pyserver_add.py  #Python
``` 

启动客户端
```sh
$ rosrun service_demo pyclient_add.py 2 3  #Python，后面可加参数
``` 


## 三种运行python节点的方式都可行
$ rosrun service_demo pyserver.py
$ rosrun service_demo pyclient.py
$ rosrun service_demo pyserver_add.py
$ rosrun service_demo pyclient_add.py
或者
$ cd /home/zqc/catkin_ws/src/ros_study/service_demo/scripts/
$ ./pyserver.py
$ ./pyclient.py
$ ./pyserver_add.py
$ ./pyclient_add.py
或者
$ cd /home/zqc/catkin_ws/src/ros_study/service_demo/scripts/
$ python pyserver.py
$ python pyclient.py
$ python pyserver_add.py
$ python pyclient_add.py

但是python pyserver.py这种方法是最方便的，不需要设置为可执行文件，不需要CMakeLists.txt设置catkin_install_python()，只要是python xxx.py都可以运行
