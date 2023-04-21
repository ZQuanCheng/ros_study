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
#include <ros/ros.h> // ROS默认头文件,python中对应的是：import rospy
// 若报错，试一试在VScode扩展中按照“ROS”


#include "ros_parameter/ParaServiceTest.h" // ParaServiceTest服务头文件（构建后自动生成）


// --------------------------------------------------------------------------------------------
// ------------------------------------相比ros_service,添加这些标识符、参数------------------------
// --------------------------------------------------------------------------------------------
#define PLUS 1 // 加
#define MINUS 2 // 减
#define MULTIPLICATION 3 // 乘
#define DIVISION 4 // 除

int g_operator = PLUS;
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------



// service回调函数，输入参数req，输出参数res， bool 返回值由于标志是否处理成功
// 当有服务请求时，会处理以下内容。
// 服务请求设置为req，服务响应设置为res。
bool calculation(ros_parameter::ParaServiceTest::Request &req,
                 ros_parameter::ParaServiceTest::Response &res)
{

    // --------------------------------------------------------------------------------------------
    // ------------------------------------相比ros_service,修改加法,变成四则运算-----------------------
    // --------------------------------------------------------------------------------------------
    // 根据g_operator参数值进行a和b的运算
    // 计算后将结果保存到服务响应值中。
    switch (g_operator)
    {
    case PLUS:
        res.result = req.a + req.b;
        break;
    case MINUS:
        res.result = req.a - req.b;
        break;
    case MULTIPLICATION:
        res.result = req.a * req.b;
        break;
    case DIVISION:
        if (req.b == 0)
        {
            res.result = 0;
            break;
        }
        else
        {
            res.result = req.a / req.b;
            break;
        }
    default:
        res.result = req.a + req.b;
        break;
    }
    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------------


    /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
    ROS_INFO和它的朋友们可用来取代printf/cout。
    ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
    这里我们使用ROS_INFO，相当于c中的printf。
    */
    // 显示服务请求中使用的a和b值，以及相当于服务响应的result值。
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.result);

    // 返回true，正确处理了请求
    return true;
}


/*
    int main(int argc, char **argv)
    第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数，在VS中默认值为1。
        比方说程序编译后，可执行文件是service_client_node。
        这时候在命令行输入rosrun ros_service service_client_node这个时候，argc的值是1。
        若是在命令行输入rosrun ros_service service_client_node para1 para2，argc的值是3。也就是 命令名 加上两个参数，一共三个参数。

    第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。各成员含义如下：
        argv[0]指向程序运行的全路径名
        argv[1]指向执行程序名后的第一个字符串
        argv[2]指向执行程序名后的第二个字符串
        argv[3]指向执行程序名后的第三个字符串
        argv[argc]为NULL
    也就是说,这个char **argv是用来取得你所输入的参数
*/
int main(int argc, char **argv) // 节点主函数
{
    /* -----------------------------------执行ros节点初始化-----------------------
    ros::init() 函数需要查看 argc 和 argv，以便它可以执行命令行提供的任何 ROS 参数和名称重映射name remapping。
    对于编程重映射，您可以使用不同版本的 init() 直接进行重新映射，但对于大多数命令行程序，传递 argc 和 argv 是最简单的方法。 init() 的第三个参数是节点的名称。
    在使用 ROS 系统的任何其他部分之前，您必须调用 ros::init() 的其中一个版本。
    */
    ros::init(argc, argv, "para_service_server"); // 解析参数，命名节点


    /* -----------------------------------为这个进程的ros节点创建句柄-----------------------
    NodeHandle 是与 ROS 系统通信的主要接入点。
    构造的第一个 NodeHandle 将完全初始化此节点，最后一个
    NodeHandle destructed 将关闭节点。
    */
    ros::NodeHandle nh; // 创建节点句柄，实例化node。
    // 声明一个节点句柄来与ROS系统进行通信
    // 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源。


    // ---------------------------------------------------------------------------------------------------------------------
    // ------------------------------------设置参数,ros::NodeHandle.setParam()-----------------------------------------------
    // ---------------------------------------------------------------------------------------------------------------------
    // 由于PLUS被#define定义为1，所以calculate_method参数变为1
    //nh.setParam("calculation_method", PLUS); // 初始化参数
    /* 函数定义：https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html#a0785b341aca36b31e1732e4618e5ae2b
    void ros::NodeHandle::setParam	(	const std::string & 	key,
                                        bool 	b 
                                    )		const
    * 在参数服务器上设置一个布尔值。
    * 函数参数:
            key: 在参数服务器的字典中使用的键。
            b: 要插入的值。
    * 我们使用setParam(key_name，value)设置关键字key_name及其对应值value，
    * 之后使用getParam(key_name，arg)从参数服务器中读取这个关键字key_name的对应值，赋值给变量arg。
    // ---------------------------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------------------------------
    // 除了ros::NodeHandle::setParam，我们还可以用ros::param::get。两者本质相同。
    // nh.setParam("calculation_method", PLUS); 
    // ros::param::set("calculation_method", PLUS);
    // ---------------------------------------------------------------------------------------------------------------------


    /* -------------------------------
    advertiseService() 函数是为了告诉 ROS：如何在给定service name上发布消息。 这将请求调用 ROS 主节点，Master主节点保留注册表，其中有谁在请求和谁在响应的信息。
    执行此 advertiseService() 调用后，主节点将通知任何试图请求此service name的人，他们将依次与此节点协商对等连接。
    advertiseService() 返回一个 ros::ServiceServer 对象，
    advertiseService() 的第二个参数决定了你的服务类型（决定了你的服务需要传入什么参数（request），传出什么参数(response)）,
    这是由第二个参数（一个函数）操作的结构体的类型（具体的结构体详情在头文件中查看）决定的。
    */
    // 声明服务服务器，创建使用ros_service功能包中的ServiceTest.srv服务文件的 服务服务器ros_para_service_server。服务名称是"ros_para_service_srv" ，
    // 第二个参数注册 回调函数calculation。此服务器当收到服务请求时，会执行calculation()。
    ros::ServiceServer ros_para_service_server = nh.advertiseService("ros_para_service_srv", calculation);
    /*
    函数定义：https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html#ae659319707eb40e8ef302763f7d632da
    ServiceServer ros::NodeHandle::advertiseService	(	const std::string & 	service,
                                                        bool(*)(MReq &, MRes &) 	srv_func
                                                    )
    此调用连接到主节点，以宣传该节点将作为服务服务器。
    */

    /* 控制台输出。用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
    ROS_INFO和它的朋友们可用来取代printf/cout。
    ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
    这里我们使用ROS_INFO，相当于c中的printf。
    */
    ROS_INFO("ready srv server!");


    // -------------------为了让service_server_node可以一直等待请求输入,我们将单独一句ros::spinOnce(); 修改为下面这样--------------------------------

    // ros::Rate对象能让你指定循环的频率。它会记录从上次调用Rate::sleep()到现在已经有多长时间，并休眠正确的时间。
    // 设定循环周期。"10"是指10Hz，是以0.1秒间隔重复
    ros::Rate loop_rate(10); // 10hz

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

        // ---------------------------------------------------------------------------------------------------------------------
        // ------------------------------------读取参数,ros::NodeHandle.getParam()-----------------------------------------------
        // ---------------------------------------------------------------------------------------------------------------------
        // 将运算符改为通过参数收到的值。
        nh.getParam("calculation_method", g_operator);
        /* 函数定义：https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html#ad25eaed5ee612a733297511aa00458e1
        bool ros::NodeHandle::getParam	(	const std::string & 	key,
                                            bool & 	b 
                                        )		const
        * 从参数服务器获取布尔值。
        * 如果您想在键不存在的情况下提供默认值，请使用 param()？？？ ros::param::set。ros::param::get？？？？
        * 函数参数:
                key: 在参数服务器的字典中使用的键。我们才能从参数服务器中读取关键字的对应值，然后赋值给第二个参数
                [out] b: 检索值的存储。
        *函数返回值return：
                如果检索到参数值，则为 true，否则为 false
        
        * 之前我们使用setParam(key_name，value)设置关键字key_name及其对应值value，
        * 现在使用getParam(key_name，arg)从参数服务器中读取这个关键字key_name的对应值，赋值给变量arg。
        */
        // ---------------------------------------------------------------------------------------------------------------------
        // ---------------------------------------------------------------------------------------------------------------------
        // ---------------------------------------------------------------------------------------------------------------------

        // ---------------------------------------------------------------------------------------------------------------------
        // 除了ros::NodeHandle::getParam，我们还可以用ros::param::set。两者本质相同。返回值是bool类型
        // nh.getParam("calculation_method", g_operator); 
        // ros::param::get("calculation_method", g_operator);
        // ---------------------------------------------------------------------------------------------------------------------



        // 问：nh.spinOnce( )和nh.spin( )是干什么的？:https://blog.csdn.net/zc15210073939/article/details/122322392
        // 如果此处没有ros::spinOnce()的话，回调函数将永远不会被调用。
        ros::spinOnce(); // 后台函数处理进程

        // 为了反复进入进程而添加的sleep（暂歇）函数
        loop_rate.sleep(); // 现在我们使用ros::Rate在剩下的时间内睡眠，以让我们达到10Hz的发布速率。  ros::Rate loop_rate(10);
    }

    return 0;
}