// 代码源于官方文档：http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// 本教程演示了通过 ROS 系统简单地发送消息。

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
#include <ros/ros.h>     // python中对应的是：import rospy
// 若报错，试一试在VScode扩展中按照“ROS”


/* ----------------------------------2、引入ros的数据流消息-----------------------
位置：/opt/ros/kinetic/include/std_msgs/String.h
<std_msgs/String.h>引用了位于std_msgs包里的std_msgs/String消息。这是从std_msgs包里的String.msg文件中自动生成的头文件。
*/
#include <std_msgs/String.h>   // python中对应的是：from std_msgs.msg import String 



// -----------------------------------3、引入sstream库，控制流-----------------------
#include <sstream>     // sstream库定义了三种类：istringstream、ostringstream和stringstream，分别用来进行流的输入、输出和输入输出操作。
// 后面会使用stringstream类，处理字符串的数据类型转换


int main(int argc, char **argv)
{
   
   /* -----------------------------------执行ros节点初始化-----------------------
   ros::init() 函数需要查看 argc 和 argv，以便它可以执行命令行提供的任何 ROS 参数和名称重映射name remapping。
   对于编程重映射，您可以使用不同版本的 init() 直接进行重新映射，但对于大多数命令行程序，传递 argc 和 argv 是最简单的方法。 init() 的第三个参数是节点的名称。
   在使用 ROS 系统的任何其他部分之前，您必须调用 ros::init() 的其中一个版本。
   */
   ros::init(argc, argv, "publisher"); //publisher是节点名称。注意：名称必须是基本名称，例如不能包含任何斜杠/。

   /* -----------------------------------为这个进程的ros节点创建句柄-----------------------
   NodeHandle 是与 ROS 系统通信的主要接入点。
   构造的第一个 NodeHandle 将完全初始化此节点，最后一个
   NodeHandle destructed 将关闭节点。
   */
   ros::NodeHandle nh;  // 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源。
 
   /* -------------------------------
   advertise() 函数是为了告诉 ROS：如何在给定topic name上发布消息。 这将请求调用 ROS 主节点，Master主节点保留注册表，其中有谁在发布和谁在订阅的信息。 
   执行此 advertise() 调用后，主节点将通知任何试图订阅此topic name的人，他们将依次与此节点协商对等连接。 
   advertise() 返回一个 ros::Publisher 对象，它允许您通过调用 publish() 发布关于该主题的消息。 一旦返回的 Publisher 对象的所有副本都被销毁，该topic将自动取消广播（unadvertised）。
   advertise() 的第二个参数是用于发布消息的消息队列的大小。 如果消息发布速度比我们发送速度快，这里的数字指定在丢弃一些消息之前要缓冲多少消息。
   */
   // 创建发布者chatter_pub; 对应话题topic name为chatter; 发布的消息类型为std_msgs/String，发布队列的大小为1000，最多缓存1000条消息。
   ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
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
   ros::Rate loop_rate(10);  // 在本例中，我们告诉它希望以10Hz运行。
   

   // 记录我们发送的消息数。 这用于为每条消息创建一个唯一的字符串。
   int count = 0;  

   /* 关于 ros::ok()
   ros::ok()在以下情况会返回false：
      1、收到SIGINT信号（Ctrl+C）。 默认情况下，roscpp将安装一个SIGINT处理程序，它能够处理Ctrl+C操作，让ros::ok()返回false。
      2、被另一个同名的节点踢出了网络
      3、ros::shutdown()被程序的另一部分调用
      3、所有的ros::NodeHandles都已被销毁
   一旦ros::ok()返回false, 所有的ROS调用都会失败。
   */
   while (ros::ok())
   {
      
     /* 定义一个std_msgs中的String类的对象，msg。
     std_msgs::String类只有一个成员：string data。具体见：http://docs.ros.org/en/api/std_msgs/html/msg/String.html
     这是一个消息对象（message object）。 我们要用数据填充它，然后发布它。
     */
     std_msgs::String msg;          
 
     // 定义一个C++标准库中的stringstream类的对象，ss。
     // std:: 是个空间标识符。c++标准库中的函数或对象都是是在命名空间std中定义的 ，所以我们使用标准函数库中的函数或对象都要使用std来限定。
     std::stringstream ss;  
 
     // 向ss中放入 1、字符串string类型的值"hello world!"; 2、放入int类型的值count
     ss << "hello world!" << count;  
 
     //将缓存流ss中的各种数据转换为string类，返回值赋给msg.data。但是并没有情况缓存流。
     msg.data = ss.str(); 

     /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
     ROS_INFO和它的朋友们可用来取代printf/cout。
     ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
     这里我们使用ROS_INFO，参数类型是const char*。但是msg.data的数据类型是string。具体见：http://docs.ros.org/en/api/std_msgs/html/msg/String.html
     那么就需要使用c_str()函数。c_str() 函数可以将C++的const string 类型 转化为 C的 const char* 类型。
     c_str() 以 char* 形式传回指针，指向字符串的首地址。注意：c_str()返回的是一个临时指针，不能对其进行操作。
     如果一个函数要求char*参数，可以使用c_str()方法。ROS_INFO（）要求的参数是char*，而不是string*，所以要.c_str()
     */
     ROS_INFO("%s", msg.data.c_str());  
     // 如果改成 ROS_INFO_STREAM(msg.data);呢？？？？？？？？？？可不可以不使用c_str()进行转换？
     // 字符串类型之间的互转：https://blog.csdn.net/Littlehero_121/article/details/105733141  https://www.cnblogs.com/mrguoguo/p/14435621.html
     

     // publish() 函数是您发送消息的方式。 参数是消息对象（message object）。
     // 此对象的类型必须与作为 advertise<>() 调用的模板参数给出的类型一致，就像在上面的构造函数中所做的那样：nh.advertise<std_msgs::String>()
     // 把msg广播给了任何已建立连接的订阅者节点。
     chatter_pub.publish(msg);
     
     // 问：nh.spinOnce( )和nh.spin( )是干什么的？:https://blog.csdn.net/zc15210073939/article/details/122322392
     // 此处调用ros::spinOnce()对于这个简单程序来说没啥必要，因为我们没有接收任何回调。
     // 然而，如果要在这个程序中添加订阅，但此处没有ros::spinOnce()的话，回调函数将永远不会被调用。所以还是加上。
     ros::spinOnce();
 
     // 现在我们使用ros::Rate在剩下的时间内睡眠，以让我们达到10Hz的发布速率。  ros::Rate loop_rate(10);
     loop_rate.sleep();
 
     //循环次数+1，发布的消息数+1
     ++count;    
   }
   
   return 0;
}

/* 对上边的内容进行一下总结：
---初始化ROS系统
---向主节点宣告我们将要在chatter话题上发布std_msgs/String类型的消息
---以每秒10次的速率向chatter循环发布消息
*/