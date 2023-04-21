#!/usr/bin/env python
#coding=utf-8

import rospy         # 相当于 #include <ros/ros.h>

import math

# 导入mgs到pkg中
# 其中msg指的是在包根目录下的msg文件夹，也即msg模块
from topic_demo.msg import Gps      # 相当于 #include <topic_demo/Gps.h>

#回调函数输入的应该是msg
def callback(Gps):
    distance = math.sqrt(math.pow(Gps.x, 2)+math.pow(Gps.y, 2)) 
    rospy.loginfo('Listener: GPS: distance=%f, state=%s', distance, Gps.state)

def listener():
    # --------------------在roscpp C++中，就是ros::init(argc, argv, "pylistener");--------------------
    # --------------------在rospy Python中，就是 rospy.init_node('pylistener')--------------------    
    rospy.init_node('pylistener', anonymous=True)
    # anonymous=True，表示后面定义相同的node名字时候，按照序号进行排列
    # ROS要求每个节点都有一个唯一的名称，如果出现具有相同名称的节点，则会与前一个节点发生冲突，
    # 这样一来，出现故障的节点很容易地被踢出网络。anonymous=True标志会告诉rospy为节点生成唯一的名称，这样就很容易可以有多个listener.py一起运行。

    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    #创建订阅者; 对应话题topic名称为gps_info; 发布的消息类型为topic_demo::Gps; 回调函数为Callback。
    # --------------------在roscpp C++中，就是ros::Subscriber sub = n.subscribe("gps_info", 100？, Callback); ----------------------
    # --------------------在rospy Python中，就是 rospy.Subscriber('gps_info', Gps, callback)--------------------
    rospy.Subscriber('gps_info', Gps, callback)

    # --------------------在roscpp C++中，就是ros::spin();--------------------
    # --------------------在rospy Python中，就是rospy.spin()-------------------- 
    rospy.spin()
    # 我记得roscpp中我总结的是topic_subsciber用spin();service_server用spinonce()
    # rospy.spin()只是不让你的节点退出，直到节点被明确关闭。
    # 与roscpp不同，rospy.spin()不影响订阅者回调函数，因为它们有自己的线程。

if __name__ == '__main__':
    listener()