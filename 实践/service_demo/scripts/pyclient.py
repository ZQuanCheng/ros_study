#!/usr/bin/env python
# coding:utf-8

# 上面的第二句指定编码类型为utf-8，是为了使python能够识别中文

import rospy         # 相当于 #include <ros/ros.h>
# 加载必需模块，注意service模块的加载方式，from 包名.srv import *
# 其中srv指的是在包根目录下的srv文件夹，也即srv模块
from service_demo.srv import *

def client_srv():
    # 初始化节点，命名为 "greetings_client"
    # --------------------在roscpp C++中，就是ros::init(argc, argv, "greetings_client");--------------------
    # --------------------在rospy Python中，就是 rospy.init_node('greetings_client')--------------------
    rospy.init_node('greetings_client')

    # 等待有可用的服务 "greetings"
    # 可以让在greetings服务可用之前一直阻塞。
    # ？？？这里的意思是等待server将服务名称"greetings"注册到master主节点？？？
    rospy.wait_for_service("greetings")

    try:
        # 定义service客户端，service名称为“greetings”，service类型为Greeting
        # --------------------在roscpp C++中，就是ros::ServiceClient greetings_client = nh.serviceClient<service_demo::Greeting>("greetings");--------------------
        # --------------------在rospy Python中，就是 greetings_client = rospy.ServiceProxy("greetings",Greeting)--------------------
        greetings_client = rospy.ServiceProxy("greetings", Greeting)

        # 向server端发送请求，发送的request内容为name和age,其值分别为"HAN", 20
        # 注意，此处发送的request内容与service文件中定义的request部分的属性是一致的
        # --------------------在C++中，就是greetings_client.call(srv) --------------------
        # --------------------在rospy Python中，就是resp = greetings_client.call(srv) -------------------- 
        resp = greetings_client.call("HAN", 20)  # greetings_client.call("HAN",20)等同于greetings_client("HAN",20)。
        # 下面这样没成功
        # srv = Greeting()
        # srv.request.name = "HANHAN" # 报错
        # srv.request.age = 200 # 报错
        # rospy.loginfo("srv.request.name:%s"%srv.request.name)
        # rospy.loginfo("srv.request.age:%s"%srv.request.age)
        # resp = greetings_client.call(srv)

        # 打印处理结果，注意调用response的方法，类似于从resp对象中调取response属性
        rospy.loginfo("Message From server:%s"%resp.feedback)
    except rospy.ServiceException as e:   # 以前是 except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

# 如果单独运行此文件，则将上面函数client_srv()作为主函数运行
if __name__=="__main__":
    client_srv()