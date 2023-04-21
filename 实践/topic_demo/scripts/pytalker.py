#!/usr/bin/env python
#coding=utf-8

import rospy         # 相当于 #include <ros/ros.h>

# 倒入自定义的数据类型
# 其中msg指的是在包根目录下的msg文件夹，也即msg模块
from topic_demo.msg import Gps      # 相当于 #include <topic_demo/Gps.h>

def talker():

    # 创建发布者pub; 对应话题（topic）名称为gps_info; 发布的消息类型为topic_demo::Gps，发布队列的大小为10，最多缓存10条消息。
    # 声明发布者，创建一个使用topic_demo功能包的Gps消息文件的 发布者pub。话题名称是"gps_info"，消息文件发布者队列（queue）的大小设置为10
    # --------------------在roscpp C++中，就是ros::Publisher pub = nh.advertise<topic_demo::Gps>("gps_info", 10);----------------------
    # --------------------在rospy Python中，就是 pub = rospy.Publisher('gps_info', Gps , queue_size=10)-------------------- 
    pub = rospy.Publisher('gps_info', Gps , queue_size=10)   
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    
    # 这里非常重要，因为它把该节点的名称告诉了rospy——只有rospy掌握了这一信息后，才会开始与ROS主节点进行通信。
    # 初始化节点,名称是pytalker。注意：名称必须是基本名称，例如不能包含任何斜杠/。
    # --------------------在roscpp C++中，就是ros::init(argc, argv, "pytalker");--------------------
    # --------------------在rospy Python中，就是 rospy.init_node('pytalker')--------------------
    rospy.init_node('pytalker', anonymous=True)   
    # argv=None —封装节点调用时传递的参数
    # anonymous=True，表示后面定义相同的node名字时候，按照序号进行排列
       # anonymous 关键字参数主要用于您通常希望其中许多节点运行并且不关心它们的名称（例如工具、GUI）的节点。 
       # 它会在节点名称的末尾添加一个随机数，以使其独一无二。 
       # 唯一名称对于像驱动程序这样的节点更重要，如果有多个节点在运行，则会出错。 
       # 如果在 ROS 图上检测到两个具有相同名称的节点，则较旧的节点将关闭。

    # 此行创建一个Rate对象 rate 。借助其方法sleep()，它提供了一种方便的方法，来以你想要的速率循环。
    # 它的参数是10，即表示希望它每秒循环10次（只要我们的处理时间不超过十分之一秒）！
    # --------------------在roscpp C++中，就是ros::Rate rate(1);--------------------
    # --------------------在rospy Python中，就是rate = rospy.Rate(1)--------------------
    rate = rospy.Rate(1)  # 更新频率是1hz

    # x=1.0
    # y=2.0
    # state='working'
    msg = Gps()
    msg.x = 10.0
    msg.y = 20.0   
    msg.state = 'working'
    # rospy.loginfo(type(msg.state))  # <type 'str'> 即数据类型是string
    rospy.loginfo('Initial Talker: GPS: x= %f, y=%f ,state= %s', msg.x, msg.y, msg.state) 
    # 不行就msg.state改成msg.state.c_str() 。   
    # 按理说需要使用c_str()函数，但是我们这里不使用c_str()才正常，使用了c_str()会报错
    # /* 控制台输出。rospy.loginfo用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
    # roscpp中的ROS_INFO和rospy中的rospy.loginfo，可用来取代printf/cout。C中的printf  C++中的cout。
    # ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM( "INFO message." <<k);相当于c++中的cout;
    # 这里我们使用rospy.loginfo，相当于roscpp中的ROS_INFO()，相当于C中的printf，"%s"的参数类型是const char*。
    # 按理说需要使用c_str()函数。c_str() 函数可以将C++的const string 类型 转化为 C的 const char* 类型。
    # c_str() 以 char* 形式传回指针，指向字符串的首地址。注意：c_str()返回的是一个临时指针，不能对其进行操作。
    # 如果一个函数要求char*参数，可以使用c_str()方法。ROS_INFO（）要求的参数是char*，而不是string*，所以要.c_str()
    # */
    # // 字符串类型之间的互转：https://blog.csdn.net/Littlehero_121/article/details/105733141  https://www.cnblogs.com/mrguoguo/p/14435621.html


    # 这个循环是一个相当标准的rospy结构：检查rospy.is_shutdown()标志，然后执行代码逻辑。你必须查看is_shutdown()以检查程序是否应该退出（例如有Ctrl+C或其他）。
    # --------------------在roscpp C++中，就是while (ros::ok())--------------------
    # --------------------在rospy Python中，就是while not rospy.is_shutdown():--------------------
    while not rospy.is_shutdown():
        
        # rospy.loginfo(str)，它有3个任务：
                                        # 打印消息到屏幕上；
                                        # 把消息写入节点的日志文件中；
                                        # 写入rosout。rosout是一个方便的调试工具：您可以使用rqt_console来拉取消息，而不必在控制台窗口找你节点的输出。
        # 计算距离
        # --------------------在roscpp C++中，就是ROS_INFO("Talker: GPS: x=%f ,y= %f", x, y); --------------------
        # --------------------在rospy Python中，就是rospy.loginfo('Talker: GPS: x=%f ,y= %f',x,y) --------------------
        # rospy.loginfo('Talker: GPS: x=%f ,y= %f', x, y)
        rospy.loginfo('Talker: GPS: x=%f ,y= %f', msg.x, msg.y)

        # 将一个Gps消息对象(state='working',x=1.0,y=2.0)发布到gps_info话题。
        # --------------------在C++中，就是pub.publish(msg); --------------------
        # --------------------在rospy Python中，就是pub.publish(msg) --------------------        
        # pub.publish(Gps(state,x,y))
        pub.publish(msg)

        # x=1.03*x
        # y=1.01*y
        msg.x = 1.03 * msg.x
        msg.y = 1.01 * msg.y 

        # 调用rate.sleep()，它在循环中可以用刚刚好的睡眠时间维持期望的速率。
        # --------------------在roscpp C++中，就是rate.sleep(); --------------------
        # --------------------在rospy Python中，就是rate.sleep() --------------------            
        rate.sleep()

# if __name__ == '__main__':
#     talker()

# 可以改成下面这样
# 除了标准的Python __main__检查，它还会捕获一个rospy.ROSInterruptException异常，
# 当按下Ctrl+C或节点因其他原因关闭时，这一异常就会被rospy.sleep()和rospy.Rate.sleep()抛出。
# 引发此异常的原因是你不会意外地在sleep()之后继续执行代码。
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass