// 代码源于：E:/机器人/2-ROS学习/ROS_Robot_Programming_CN.pdf  7.3 创建和运行服务服务器与客户端节点
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


#include "ros_service/ServiceTest.h" // ServiceTest服务头文件（构建后自动生成）


#include <cstdlib>     // 使用atoll函数所需的库


/*
    int main(int argc, char **argv)
    第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数，在VS中默认值为1,因为可执行程序名也是一个参数。
        比方说程序编译后，可执行文件是service_client_node。
        这时候在命令行输入rosrun ros_service service_client_node这个时候，argc的值是1。
        若是在命令行输入rosrun ros_service service_client_node arg0 arg1，argc的值是3。也就是 命令名 加上两个参数，一共三个参数。

    第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。各成员含义如下： 
        argv[0]指向程序运行的全路径名 
        argv[1]指向执行程序名后的第一个字符串 
        argv[2]指向执行程序名后的第二个字符串 
        argv[3]指向执行程序名后的第三个字符串 
        argv[argc]为NULL 
    也就是说,这个char **argv是用来获取你所输入的参数
*/
int main(int argc, char **argv)                // 节点主函数
{
    /* -----------------------------------执行ros节点初始化-----------------------
    ros::init() 函数需要查看 argc 和 argv，以便它可以执行命令行提供的任何 ROS 参数和名称重映射name remapping。
    对于编程重映射，您可以使用不同版本的 init() 直接进行重新映射，但对于大多数命令行程序，传递 argc 和 argv 是最简单的方法。 init() 的第三个参数是节点的名称。
    在使用 ROS 系统的任何其他部分之前，您必须调用 ros::init() 的其中一个版本。
    */
    ros::init(argc, argv, "service_client"); // 解析参数，命名节点


    /*
    例子:当我们在命令行输入rosrun ros_service service_client_node 2 3时,
    这时候service_client.cpp的main(int argc, char **argv)函数会读取命令
    argc统计命令行总的参数个数: 可执行文件名作为一个参数+后面的参数个数,即argc=3
    argv获取你所输入的参数: 
            argv[0]指向程序运行的全路径名; 
            argv[1]指向执行程序名后的第一个字符串, 即argv[1]='2'; 然后用atoll() 函数, atoll(argv[1])将字符串值'2'转换为整数值2.
            argv[2]指向执行程序名后的第二个字符串, 即argv[2]='3'; 然后用atoll() 函数, atoll(argv[2])将字符串值'3'转换为整数值3.
            argv[argc]为NULL, 即argv[3]=NULL;
            
    之后服务请求得到回应, 收到整数值5.
    */
    // 这里我们要求rosrun ros_service service_client_node 后面跟的必须是两个参数,用于加法计算.
    // 这样的话argc统计命令行总的参数个数: 命令名+后面的两个参数,即 argc=3
    // 如果argc!=3,说明rosrun ros_service service_client_node 后面跟的参数个数不符合要求,需要提示
    if (argc != 3)                           // 处理输入值错误
    {
        /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
        ROS_INFO和它的朋友们可用来取代printf/cout。
        ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
        这里我们使用ROS_INFO，相当于c中的printf。
        */  
        // argc!=3,说明rosrun ros_service service_client_node 后面跟的参数个数不符合要求,需要提示输入两个数字 arg0 arg1
        ROS_INFO("cmd : rosrun ros_service service_client arg0 arg1");
        ROS_INFO("arg0: double number, arg1: double number");
        return 1;
    }


    /* -----------------------------------为这个进程的ros节点创建句柄-----------------------
    NodeHandle 是与 ROS 系统通信的主要接入点。
    构造的第一个 NodeHandle 将完全初始化此节点，最后一个
    NodeHandle destructed 将关闭节点。
    */
    ros::NodeHandle nh;  // 创建节点句柄，实例化node。
    // 声明一个节点句柄来与ROS系统进行通信
    // 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源。


    /* -------------------------------
    serviceClient() 函数是为了告诉 ROS：如何接收给定service name的响应消息。 这将请求调用 ROS 主节点，Master主节点保留注册表，其中有谁在请求和谁在响应的信息。  
    serviceClient() 返回一个 ros::ServiceClient 对象，
    */
    // 声明客户端，声明利用ros_service功能包的ServiceTest.srv服务文件的 服务客户端ros_service_client。服务名称是"ros_tutorial_srv"
    ros::ServiceClient ros_service_client = nh.serviceClient<ros_service::ServiceTest>("ros_service_srv");
    /*
    函数定义：https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html#aa3376eeca609c4985255cecfaadcbcc5	        
    ServiceClient ros::NodeHandle::serviceClient	(	const std::string & 	service_name,
                                                        bool 	persistent = false,
                                                        const M_string & 	header_values = M_string() 
                                                    )	                                                                            
    此调用连接到主节点，该节点将作为服务客户端。
    */ 


    // 声明一个使用ServiceTest服务文件的叫做srv的服务
    ros_service::ServiceTest srv;
    

    /*
    例子:当我们在命令行输入rosrun ros_service service_client_node 2 3时,
    这时候service_client.cpp的main(int argc, char **argv)函数会读取命令
    argc统计命令行总的参数个数: 可执行文件名作为一个参数+后面的参数,即argc=3
    argv获取你所输入的参数: 
            argv[0]指向程序运行的全路径名; 
            argv[1]指向执行程序名后的第一个字符串, 即argv[1]='2'; 然后用atoll() 函数, atoll(argv[1])将字符串值'2'转换为整数值2.
            argv[2]指向执行程序名后的第二个字符串, 即argv[2]='3'; 然后用atoll() 函数, atoll(argv[2])将字符串值'3'转换为整数值3.
            argv[argc]为NULL, 即argv[3]=NULL;

    之后服务请求得到回应, 收到整数值5.
    */
    // 在执行服务客户端节点时用作输入的参数分别保存在a和b中
    // atoll() 函数是 cstdlib 头文件的库函数。它用于将给定的字符串值转换为整数值。它接受一个包含整数(整数)数的字符串并返回其 long long long 整数值。
    srv.request.a = atoll(argv[1]);    
    srv.request.b = atoll(argv[2]);
    /*
    int main(int argc, char **argv)
    第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数，在VS中默认值为1,因为可执行程序名也是一个参数。
        比方说程序编译后，可执行文件是service_client_node。
        这时候在命令行输入rosrun ros_service service_client_node这个时候，argc的值是1。
        若是在命令行输入rosrun ros_service service_client_node arg0 arg1，argc的值是3。也就是 命令名 加上两个参数，一共三个参数。

    第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。各成员含义如下： 
        argv[0]指向程序运行的全路径名 
        argv[1]指向执行程序名后的第一个字符串 
        argv[2]指向执行程序名后的第二个字符串 
        argv[3]指向执行程序名后的第三个字符串 
        argv[argc]为NULL 
    也就是说,这个char **argv是用来获取你所输入的参数
    */


    // 请求服务，如果请求被接受，则显示响应值
    if (ros_service_client.call(srv))        // 服务服务器中有回调函数，输入参数req，输出参数res， bool 返回值由于标志是否处理成功
    {
        ROS_INFO("send srv, srv.Request.a and b: %ld, %ld", (long int)srv.request.a, (long int)srv.request.b);
        ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.result);
    }
    else
    {
        ROS_ERROR("Failed to call service ros_service_srv");
        return 1;
    }
    /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
    ROS_INFO和它的朋友们可用来取代printf/cout。
    ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
    这里我们使用ROS_INFO，相当于c中的printf。
    */      
    
    return 0;
}