// 代码源于：E:/机器人/2-ROS学习/ROS_Robot_Programming_CN.pdf  7.2 发布者节点和订阅者节点的创建和运行
/*
注意，使用ROS消息和服务时，放置在/msg和/srv目录中的消息、服务和动作文件的名称遵循CamelCased 规则。
这是因为*.msg、*.srv和*.action被转换为头文件后用作结构体和数据类型（例如TransformStamped.msg，SetSpeed.srv）。
*/


/* -----------------------------------1、引入ros头文件-----------------------
位置：/opt/ros/kinetic/include/ros/ros.h
<ros/ros.h>是一个很便利的include，它包括了使用ROS系统中最常见的公共部分所需的全部头文件。
链接：https://blog.csdn.net/chanleoo/article/details/80583349
头文件<ros/ros.h>里包括以下功能函数：
ros::init() : 解析传入的ROS参数，从而初始化节点的名称和其他信息，一般我们ROS程序一开始都会以这种方式开始。
ros::NodeHandle : 和topic、service、param等交互的公共接口，节点的句柄(相当于５１单片机里边的ＥＡ)，只有有了它才可以用来创建Publisher、Subscriber以及用来对当前节点进行各种操作。
ros::master : 包含从master查询信息的函数
ros::this_node：包含查询这个进程(node)的函数
ros::service：包含查询服务的函数
ros::param：包含查询参数服务器的函数，而不需要用到NodeHandle
ros::names：包含处理ROS图资源名称的函数
ros::shutdown()手动关闭rosnode／ctrl+c  
*/
#include <ros/ros.h>     // ROS默认头文件,python中对应的是：import rospy
// 若报错，试一试在VScode扩展中按照“ROS”

#include "ros_topic/TopicTest.h" // topic_test消息头文件（构建后自动生成）

/*
这是一个回调函数，当有新消息到达ros_topic_msg话题时它就会被调用。
该消息是用boost shared_ptr智能指针传递的，这意味着你可以根据需要存储它，即不用担心它在下面被删除，又不必复制底层（underlying）数据。
*/
// 这是一个消息后台函数，
// 此函数在收到一个下面设置的名为ros_topic_msg的话题时候被调用。
// 输入的消息是从ros_topic功能包接收TopicTest消息。
void msgCallback(const ros_topic::TopicTest::ConstPtr& msg)
{
    /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
    ROS_INFO和它的朋友们可用来取代printf/cout。
    ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
    这里我们使用ROS_INFO，相当于c中的printf。
    */
    /* 显示stamp.sec消息、stamp.nsec消息、data消息。
    这是不是说明msg.stamp.sec和stamp.nsec的数据类型都是int？？？
    但是官网没看到time类型的数据还可以细分啊？https://docs.ros.org/en/api/std_msgs/html/msg/Time.html
    这个博客有ros::Time：https://blog.csdn.net/xq151750111/article/details/114631573
    具体来说，roscpp里有两种时间的表示方法，一种是时刻（ros::Time），一种是时长（ros::Duration）。无论是Time还是Duration都具有相同的表示方法：
    Times/Duration都由秒(sec)和纳秒(nsec)组成。
    ···
    int32 sec
    int32 nsec
    ···    
    */
    ROS_INFO("recieve msg = %d", msg->stamp.sec); // 显示stamp.sec消息
    ROS_INFO("recieve msg = %d", msg->stamp.nsec); // 显示stamp.nsec消息
    ROS_INFO("recieve msg = %d", msg->data); // 显示data消息
}

int main(int argc, char **argv) // 节点主函数
{
    /* -----------------------------------执行ros节点初始化-----------------------
    ros::init() 函数需要查看 argc 和 argv，以便它可以执行命令行提供的任何 ROS 参数和名称重映射name remapping。
    对于编程重映射，您可以使用不同版本的 init() 直接进行重新映射，但对于大多数命令行程序，传递 argc 和 argv 是最简单的方法。 init() 的第三个参数是节点的名称。
    在使用 ROS 系统的任何其他部分之前，您必须调用 ros::init() 的其中一个版本。
    */
    ros::init(argc, argv, "topic_subscriber"); // 初始化节点,名称是topic_subscriber。注意：名称必须是基本名称，例如不能包含任何斜杠/。

    /* -----------------------------------为这个进程的ros节点创建句柄-----------------------
    NodeHandle 是与 ROS 系统通信的主要接入点。
    构造的第一个 NodeHandle 将完全初始化此节点，最后一个
    NodeHandle destructed 将关闭节点。
    */
    ros::NodeHandle nh; // 声明用于ROS系统和通信的节点句柄
    // 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源。

    /**
    调用subscribe() 是告诉 ROS :如何接收关于给定topic name的消息。 这将请求调用 ROS 主节点，Master主节点保留注册表，其中有谁在发布和谁在订阅的信息。 
    消息被传递到回调函数，这里称为 msgCallback。 每当有新消息到达时，ROS将调用msgCallback()函数。
    subscribe() 返回一个 ros::Subscriber 对象，您必须持有该对象直到您想要取消订阅（unsubscribe）。 
    当 Subscriber 对象的所有副本超出范围时，此回调将自动取消对该主题的订阅。
    当 Subscriber 对象被析构，它将自动从chatter话题取消订阅。
    析构函数(destructor) 与构造函数相反，当对象结束其生命周期，如对象所在的函数已调用完毕时，系统自动执行析构函数。析构函数往往用来做“清理善后” 的工作

    subscribe() 函数的第二个参数是消息队列的大小。 如果消息到达的速度比处理它们的速度快，这是在开始丢弃最旧的消息之前，将被缓冲的消息数。
    */
    // 创建订阅者ros_topic_pub; 对应话题topic name为ros_topic_msg; 接收队列的大小为100，最多缓存100条消息; 回调函数为msgCallback。
    // 声明订阅者，创建一个订阅者ros_tutorial_sub，它利用ros_topic功能包的的TopicTest消息文件。话题名称是"ros_topic_msg"，订阅者队列（queue)的大小设为100。
    ros::Subscriber ros_topic_pub = nh.subscribe("ros_topic_msg", 100, msgCallback);
    /*
    函数定义：https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html#a302620aff50f66c4b73fc613a55c27aa
    Subscriber ros::NodeHandle::subscribe	(	const std::string & 	topic,
                                          uint32_t 	queue_size,
                                          const boost::function< void(C)> & 	callback,
                                          const VoidConstPtr & 	tracked_object = VoidConstPtr(),
                                          const TransportHints & 	transport_hints = TransportHints() 
                                        )		
  
    */

    /**
    ros::spin() 将进入一个循环，pump回调callbacks。 在这个版本中，所有回调(callbacks)都将从这个线程（主线程）中调用。 
    ros::spin() 将在按下 Ctrl-C 或节点被 master 关闭时退出。
    */
    ros::spin(); // 用于调用回调函数，等待接收消息。在接收到消息时执行回调函数。
    /*
    ros::spin()启动了一个自循环，它会尽可能快地调用消息回调函数。不过不要担心，如果没有什么事情，它就不会占用太多CPU。
    另外，一旦ros::ok()返回false，ros::spin()就会退出，这意味着ros::shutdown()被调用了，主节点让我们关闭（或是因为按下Ctrl+C，它被手动调用）。
    */
  
    /* 关于 ros::ok()
     ros::ok()在以下情况会返回false：
        1、收到SIGINT信号（Ctrl+C）。 默认情况下，roscpp将安装一个SIGINT处理程序，它能够处理Ctrl+C操作，让ros::ok()返回false。
        2、被另一个同名的节点踢出了网络
        3、ros::shutdown()被程序的另一部分调用
        4、所有的ros::NodeHandles都已被销毁
     一旦ros::ok()返回false, 所有的ROS调用都会失败。
    */

    return 0;
}