# ros_topic

ros_topic软件包，包括C++版本的Topic通信的示例。

## 功能介绍

Topic的发布者以**10HZ**的频率向**/ros_topic_msg**这个topic上发布消息，消息格式要包括系统时间(stamp)和已发送消息数(data)。

Topic的接受者会订阅**/ros_topic_msg**，并在屏幕上显示。

本例需要自定义msg文件，见[msg/TopicTest.msg](./msg/TopicTest.msg)。

C++版本代码见`src/`下的[topic_publisher.cpp](./src/topic_publisher.cpp)和[topic_subscriber.cpp](./src/topic_subscriber.cpp)。

Python版本代码(无)。


## 运行方法

启动发布者

```sh
$ rosrun ros_topic topic_publisher_node        #C++
``` 

启动接收者

```sh
$ rosrun ros_topic topic_subscriber_node        #C++
``` 

msg是与编程语言无关的通信协议，因此收发双方无论用哪个语言来实现，都可以实现相互的topic通信。
