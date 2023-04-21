#! /usr/bin/env python
# coding=utf-8

"""
    文件开头指定解释器
    ros版的python helloworld
"""

# 引入包
import rospy

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node("hello_python_node")
    # 打印日志helloworld
    rospy.loginfo("ROS Hello World!!!! by python")