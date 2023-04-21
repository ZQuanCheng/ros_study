#!/usr/bin/env python
# coding:utf-8

# 源代码：https://wiki.ros.org/cn/ROS/Tutorials/WritingServiceClient%28python%29

# https://blog.csdn.net/The_Time_Runner/article/details/89598051
# from __future__ import print_function

import rospy        

from service_demo.srv import *

 
def handle_add_two_ints(req):
    # 显示输出
    # rospy.loginfo("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))

    # 返回Rosponse
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    # 初始化节点，命名为 "greetings_server"
    rospy.init_node('add_two_ints_server')
    # 定义service的server端，service名称为"add_two_ints"， service类型为AddTwoInts, 收到的request请求信息将作为参数传递给回调函数handle_add_two_ints进行处理
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    # 显示输出
    # rospy.loginfo("Ready to add two ints.")
    print("Ready to add two ints.")

    rospy.spin()
 
if __name__ == "__main__":
    add_two_ints_server()