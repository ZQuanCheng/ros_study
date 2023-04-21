#!/usr/bin/env python
# coding:utf-8

# 上面指定编码utf-8，使python能够识别中文


import rospy         # 相当于 #include <ros/ros.h>

# 加载必需模块，注意service模块的加载方式，from 包名.srv import *
# 其中srv指的是在包根目录下的srv文件夹，也即srv模块
from service_demo.srv import *
# from…import *：是把一个模块中所有函数都导入进来; 注：相当于：相当于导入的是一个文件夹中所有文件，所有函数都是绝对路径。
# 结论：
# from…import *语句与import区别在于：
# import 导入模块，每次使用模块中的函数都要是定是哪个模块。
# from…import * 导入模块，每次使用模块中的函数，直接使用函数就可以了；注因为已经知道该函数是那个模块中的了。

def server_srv():
    # 初始化节点，命名为 "greetings_server"
    # --------------------在roscpp C++中，就是ros::init(argc, argv, "greetings_server");--------------------
    # --------------------在rospy Python中，就是 rospy.init_node('greetings_server')--------------------
    rospy.init_node("greetings_server")

    # 定义service的server端，service名称为"greetings"， service类型为Greeting, 收到的request请求信息将作为参数传递给回调函数handle_function进行处理
    # --------------------在roscpp C++中，就是ros::ServiceServer s = nh.advertiseService("gps_info", handle_function);----------------------
    # --------------------在rospy Python中，就是 s = rospy.Service("greetings", Greeting, handle_function)--------------------     
    s = rospy.Service("greetings", Greeting, handle_function)

    #   /* 控制台输出。rospy.loginfo用来取代printf/cout。见文档：http://wiki.ros.org/rosconsole
    #   roscpp中的ROS_INFO和rospy中的rospy.loginfo，可用来取代printf/cout。C中的printf  C++中的cout。
    #   ROS_INFO(“INFO message %d”,k),相当于c中的printf;  ROS_INFO_STREAM ( "INFO message." <<k);相当于c++中的cout;
    #   这里我们使用rospy.loginfo，相当于roscpp中的ROS_INFO()，相当于C中的printf，"%s"的参数类型是const char*。
    #   */
    # --------------------在roscpp C++中，就是 ROS_INFO("Ready to handle the request:");--------------------
    # --------------------在rospy Python中，就是 rospy.loginfo("Ready to handle the request:")--------------------    
    rospy.loginfo("Ready to handle the request:")


    # 阻塞程序结束
    # --------------------在roscpp C++中，就是ros::spin();--------------------
    # --------------------在rospy Python中，就是rospy.spin()-------------------- 
    rospy.spin()
    # 我记得roscpp中我总结的是topic_subsciber用spin();service_server用spinonce()
    # rospy.spin()只是不让你的节点退出，直到节点被明确关闭。
    # 与roscpp不同，rospy.spin()不影响订阅者回调函数，因为它们有自己的线程。


# Define the handle function to handle the request inputs
# 最开始 from service_demo.srv import * 导入模块，每次使用模块中的函数，直接使用函数就可以了；注因为已经知道该函数是那个模块中的了。
def handle_function(req):  
    # 注意我们是如何调用request请求内容的，与前面client端相似，都是将其认为是一个对象的属性，通过对象调用属性，在我们定义
    # 的Service_demo类型的service中，request部分的内容包含两个变量，一个是字符串类型的name，另外一个是整数类型的age
    rospy.loginfo( 'Request from %s with age %d'%(req.name, req.age))
    
    # 返回一个Service_demoResponse实例化对象，其实就是返回一个response的对象，其包含的内容为我们再Service_demo.srv中定义的
    # response部分的内容，我们定义了一个string类型的变量，因此，此处实例化时传入字符串即可
    return GreetingResponse("Hi %s. I' server!"%req.name)


# 如果单独运行此文件，则将上面定义的server_srv作为主函数运行
if __name__=="__main__":
    server_srv()