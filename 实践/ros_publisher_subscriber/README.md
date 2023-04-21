# ros_publisher_subscriber

ros_publisher_subscriber软件包，包括C++版本的Topic通信的示例，但是没有自定义msg文件。

## 功能介绍

Topic的发布者以**10HZ**的频率向**/chatter**这个topic上发布消息，消息格式为std_msgs/String类型。

Topic的接受者会订阅**/chatter**，并在屏幕上显示。

本例不需要自定义msg文件，直接使用std_msgs/String消息类型。

C++版本代码见`src/`下的[publisher.cpp](./src/publisher.cpp)和[subscriber.cpp](./src/subscriber.cpp)。

Python版本代码(无)。


## 运行方法

启动发布者

```sh
$ rosrun ros_publisher_subscriber publisher_node        #C++
``` 

启动接收者

```sh
$ rosrun topic_demo subscriber_node        #C++
``` 

msg是与编程语言无关的通信协议，因此收发双方无论用哪个语言来实现，都可以实现相互的topic通信。
