# ros_service

ros_service软件包，包括C++与Python两个版本的service通信的示例。

## 功能介绍

假设Service的提供方/服务器端提供名为`ros_service_srv`的服务，其srv的请求是a和b，反馈为加法result给请求方。

本例需要自定义srv文件，见[srv/ServiceTest.srv](./srv/ServiceTest.srv)。

C++版本代码见`src/`下的[service_client.cpp](./src/service_client.cpp)和[service_server.cpp](./src/service_server.cpp)。

Python版本代码(无)。


## 运行方法

启动发布者

```sh
$ rosrun ros_service service_server_node        #C++
``` 

启动接收者

```sh
$ rosrun ros_service service_client_node        #C++
``` 
