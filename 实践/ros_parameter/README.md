# ros_parameter

ros_parameter软件包，包括C++与Python两个版本的parameter相关操作。

ros_parameter是在ros_service上进行修改的。

## 功能介绍

假设Service的提供方/服务器端提供名为`ros_service_srv`的服务，其srv的请求是a和b，反馈为result给请求方。

我们可以通过设置参数服务器中的参数/calculation_method，来选择四则运算。

本例需要自定义srv文件，见[srv/ParaServiceTest.srv](./srv/ParaServiceTest.srv)。

C++版本代码见`src/`下的[para_service_client.cpp](./src/para_service_client.cpp)和[para_service_server.cpp](./src/para_service_server.cpp)。

Python版本代码(无)。

本例的launch文件可以加载param.yaml参数文件，见[param/param.yaml](./param/param.yaml)。

## 运行方法

启动发布者

```sh
$ rosrun ros_parameter para_service_server_node        #C++
``` 

选择一种运算
```sh
$ rosparam set /calculation_method 1  #加
$ rosparam set /calculation_method 2  #减
$ rosparam set /calculation_method 3  #乘
$ rosparam set /calculation_method 4  #除
``` 

启动接收者
```sh
$ rosrun ros_parameter para_service_client_node 10 5       #C++ ，后面可加参数
``` 


```sh
$ roslaunch ros_parameter union.launch      #launch文件中加载参数，启动节点
``` 