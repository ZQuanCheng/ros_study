// 代码源于：E:/机器人/2-ROS学习/ROS_Robot_Programming_CN.pdf  7.4 创建和运行动作服务器与客户端节点
/*
注意，使用ROS消息和服务时，放置在/msg和/srv目录中的消息、服务和动作文件的名称遵循CamelCased 规则。
这是因为*.msg、*.srv和*.action被转换为头文件后用作结构体和数据类型（例如TransformStamped.msg，SetSpeed.srv）。
*/

#include <ros/ros.h>                               // ROS的基本头文件

#include <actionlib/client/simple_action_client.h> // 动作库头文件

#include <actionlib/client/terminal_state.h>       // 动作目标状态头文件

#include <ros_action/TestAction.h>  // TestAction动作头文件（构建后自动生成）


int main(int argc, char **argv)                    // 节点主函数
{
    ros::init(argc, argv, "action_client"); // 初始化节点名称
    // 声明动作客户端（动作名称：ros_action_test）
    actionlib::SimpleActionClient<ros_action::TestAction> ac("ros_action_test", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // 等待动作服务器启动
    ROS_INFO("Action server started, sending goal.");
    ros_action::TestGoal goal; // 声明动作目标
    goal.order = 20;                          // 指定动作目标（进行20次斐波那契运算）
    ac.sendGoal(goal);                        // 发送动作目标
    // 设置动作完成时间限制（这里设置为30秒）
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    // 在动作完成时限内收到动作结果值时
    if (finished_before_timeout)
    {
        // 获取动作目标状态值并将其显示在屏幕上
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out."); // 超过了动作完成时限的情况
    // exit
    return 0;
}