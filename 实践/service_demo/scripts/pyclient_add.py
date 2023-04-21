#!/usr/bin/env python
# coding:utf-8

# 源代码：https://wiki.ros.org/cn/ROS/Tutorials/WritingServiceClient%28python%29

# https://blog.csdn.net/The_Time_Runner/article/details/89598051
# from __future__ import print_function

import sys
import rospy
from service_demo.srv import *

def add_two_ints_client(x, y):
    # 等待有可用的服务 'add_two_ints'
    # 可以让在add_two_ints服务可用之前一直阻塞。
    # ？？？这里的意思是等待server将服务名称'add_two_ints'注册到master主节点？？？
    rospy.wait_for_service('add_two_ints')
    try:
        # 定义service客户端，service名称为“add_two_ints”，service类型为AddTwoInts
        # --------------------在roscpp C++中，就是ros::ServiceClient add_two_ints = nh.serviceClient<service_demo::AddTwoInts>("add_two_ints");--------------------
        # --------------------在rospy Python中，就是 add_two_ints = rospy.ServiceProxy("add_two_ints",AddTwoInts)--------------------        
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        
        # 向server端发送请求，发送的request内容为request.a和request.b,其值分别为x, y
        # 注意，此处发送的request内容与service文件中定义的request部分的属性是一致的
        # --------------------在C++中，就是add_two_ints.call(srv) --------------------
        # --------------------在rospy Python中，就是resp = add_two_ints.call(srv) --------------------
        resp = add_two_ints(x, y) # add_two_ints.call(x, y)等同于add_two_ints(x, y)。
        return resp.sum
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)
 
def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    # 当我们在命令行输入python pyclient_add.py 2 3时,
    # Python语言没有argc，只有argv
    # sys.argv获取你所输入的参数: 
    #         sys.argv[0]指向程序运行的全路径名; 
    #         sys.argv[1]指向执行程序名后的第一个字符串, 即argv[1]='2'; 然后用atoll() 函数, atoll(argv[1])将字符串值'2'转换为整数值2.
    #         sys.argv[2]指向执行程序名后的第二个字符串, 即argv[2]='3'; 然后用atoll() 函数, atoll(argv[2])将字符串值'3'转换为整数值3.
    #         sys.argv[argc]为NULL, 即argv[3]=NULL;   
    #  之后服务请求得到回应, 收到整数值5.
    #  --------------------在roscpp C++中，就是if (argc == 3){}--------------------
    #  --------------------在rospy Python中，就是 if len(sys.argv) == 3:--------------------  
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:                        # // 如果len(sys.argv)!=3,说明python pyclient_add.py 后面跟的参数个数不符合要求,需要提示
        # --------------------在roscpp C++中，就是 ROS_INFO(usage());--------------------
        # --------------------在rospy Python中，就是 rospy.loginfo(usage())--------------------   
        # rospy.loginfo(usage()) 
        # 用rospy.loginfo不显示，不知道为什么
        print(usage()) 

        # sys.exit()会引发一个异常：SystemExit，如果这个异常没有被捕获，那么python解释器将会退出。
        # 如果有捕获此异常的代码，那么这些代码还是会执行。捕获这个异常可以做一些额外的清理工作。
        # 0为正常退出，其他数值（1-127）为不正常，可抛异常事件供捕获。
        sys.exit(1)

    # rospy.loginfo("Requesting %s+%s"%(x, y))
    # 用rospy.loginfo不显示，不知道为什么
    print("Requesting %s+%s"%(x, y))  

    # 调用def add_two_ints_client(x, y)
    # rospy.loginfo("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
    # 用rospy.loginfo不显示，不知道为什么
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))