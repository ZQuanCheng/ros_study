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


int main(int argc, char **argv) // 节点主函数
{
    /* -----------------------------------执行ros节点初始化-----------------------
    ros::init() 函数需要查看 argc 和 argv，以便它可以执行命令行提供的任何 ROS 参数和名称重映射name remapping。
    对于编程重映射，您可以使用不同版本的 init() 直接进行重新映射，但对于大多数命令行程序，传递 argc 和 argv 是最简单的方法。 init() 的第三个参数是节点的名称。
    在使用 ROS 系统的任何其他部分之前，您必须调用 ros::init() 的其中一个版本。
    */
    ros::init(argc, argv, "topic_publisher"); // 初始化节点,名称是topic_publisher。注意：名称必须是基本名称，例如不能包含任何斜杠/。

    /* -----------------------------------为这个进程的ros节点创建句柄-----------------------
    NodeHandle 是与 ROS 系统通信的主要接入点。
    构造的第一个 NodeHandle 将完全初始化此节点，最后一个
    NodeHandle destructed 将关闭节点。
    */
    ros::NodeHandle nh; // 声明一个节点句柄来与ROS系统进行通信
    // 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源。


    /* -------------------------------
    advertise() 函数是为了告诉 ROS：如何在给定topic name上发布消息。 这将请求调用 ROS 主节点，Master主节点保留注册表，其中有谁在发布和谁在订阅的信息。 
    执行此 advertise() 调用后，主节点将通知任何试图订阅此topic name的人，他们将依次与此节点协商对等连接。 
    advertise() 返回一个 ros::Publisher 对象，它允许您通过调用 publish() 发布关于该主题的消息。 一旦返回的 Publisher 对象的所有副本都被销毁，该topic将自动取消广播（unadvertised）。
    advertise() 的第二个参数是用于发布消息的消息队列的大小。 如果消息发布速度比我们发送速度快，这里的数字指定在丢弃一些消息之前要缓冲多少消息。
    */
    // 创建发布者ros_topic_pub; 对应话题topic name为chatter; 发布的消息类型为ros_topic::TopicTest，发布队列的大小为100，最多缓存100条消息。
    // 声明发布者，创建一个使用ros_topic功能包的TopicTest消息文件的发布者ros_topic_pub。话题名称是"ros_topic_msg"，消息文件发布者队列（queue）的大小设置为100 
    ros::Publisher ros_topic_pub = nh.advertise<ros_topic::TopicTest>("ros_topic_msg", 100);
    /*
    函数定义：https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html#a6b655c04f32c4c967d49799ff9312ac6 	
    Publisher ros::NodeHandle::advertise	(	const std::string & 	topic,
                                            uint32_t 	queue_size,
                                            bool 	latch = false 
                                        )	
    此调用连接到主节点，以宣传该节点将发布关于给定topic的消息。
    NodeHandle::advertise()返回一个ros::Publisher对象（发布服务器），它有2个目的：
    其一，它包含一个publish()方法，可以将消息发布到创建它的话题上；
    其二，当超出范围时，它将自动取消这一宣告操作。
    */ 


    // ros::Rate对象能让你指定循环的频率。它会记录从上次调用Rate::sleep()到现在已经有多长时间，并休眠正确的时间。
    // 设定循环周期。"10"是指10Hz，是以0.1秒间隔重复
    ros::Rate loop_rate(10);
    
    
    /* 定义一个ros_topic功能包中的TopicTest类的对象，msg。
    ros_topic::TopicTest类有两个成员：time stamp、int32 data。
    这是一个消息对象（message object）。 我们要用数据填充它，然后发布它。
    */
    // 以TopicTest.msg消息文件格式声明一个叫做msg的消息
    ros_topic::TopicTest msg; 
    
    // 记录我们发送的消息数。 这用于为每条消息创建一个唯一的字符串。
    int count = 0; // 声明要在消息中使用的变量


    /* 关于 ros::ok()
    ros::ok()在以下情况会返回false：
       1、收到SIGINT信号（Ctrl+C）。 默认情况下，roscpp将安装一个SIGINT处理程序，它能够处理Ctrl+C操作，让ros::ok()返回false。
       2、被另一个同名的节点踢出了网络
       3、ros::shutdown()被程序的另一部分调用
       4、所有的ros::NodeHandles都已被销毁
    一旦ros::ok()返回false, 所有的ROS调用都会失败。
    */
    while (ros::ok())
    {
        //ros::Time::now()输出系统时间，返回值赋给msg.stamp。
        //msg.stamp的数据类型是time。具体见：https://docs.ros.org/en/api/std_msgs/html/msg/Time.html
        msg.stamp = ros::Time::now(); 
        
        // 将变量count的值传给msg.data
        // msg.data的数据类型是int。
        msg.data = count; 

        /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
        ROS_INFO和它的朋友们可用来取代printf/cout。
        ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
        这里我们使用ROS_INFO，相当于c中的printf。
        */
        /* 显示stamp.sec消息、stamp.nsec消息、data消息。
        这是不是说明msg.stamp.sec和stamp.nsec的数据类型都是int？？？
        但是官网没看到time类型的数据还可以细分啊？https://docs.ros.org/en/api/std_msgs/html/msg/Time.html
        */
        ROS_INFO("send msg = %d", msg.stamp.sec);  // 显示stamp.sec消息。
        ROS_INFO("send msg = %d", msg.stamp.nsec); // 显示stamp.nsec消息。
        ROS_INFO("send msg = %d", msg.data); // 显示data消息

        /* publish() 函数是您发送消息的方式。 参数是消息对象（message object）。
        此对象的类型必须与作为 advertise<>() 调用的模板参数给出的类型一致，就像在上面的构造函数中所做的那样：nh.advertise<ros_topic::TopicTest>()
        把msg广播给了任何已建立连接的订阅者节点。
        */
        ros_topic_pub.publish(msg); // 发布消息。

        // ros::spinOnce(); // topic_publisher这里没有回调函数callback()，不需要ros::spinOnce(); 

        loop_rate.sleep(); // 现在我们使用ros::Rate在剩下的时间内睡眠，以让我们达到10Hz的发布速率。  ros::Rate loop_rate(10);

        ++count; //循环次数+1，发布的消息数+1
    }
    
    return 0;
}