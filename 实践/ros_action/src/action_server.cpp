// 代码源于：E:/机器人/2-ROS学习/ROS_Robot_Programming_CN.pdf  7.4 创建和运行动作服务器与客户端节点
/*
注意，使用ROS消息和服务时，放置在/msg和/srv目录中的消息、服务和动作文件的名称遵循CamelCased 规则。
这是因为*.msg、*.srv和*.action被转换为头文件后用作结构体和数据类型（例如TransformStamped.msg，SetSpeed.srv）。
*/

#include <ros/ros.h>                               // ROS的基本头文件

#include <actionlib/server/simple_action_server.h> // 动作库头文件

#include <ros_action/TestAction.h>  // TestAction动作头文件（生成后自动生成）

class FibonacciAction
{
protected:
    // 声明节点句柄
    ros::NodeHandle nh_;
    // 声明动作服务器
    actionlib::SimpleActionServer<ros_action::TestAction> as_;
    // 用作动作名称
    std::string action_name_;
    // 声明用于发布的反馈及结果
    ros_action::TestFeedback feedback_;
    ros_action::TestResult result_;

public:
    // 初始化动作服务器（节点句柄、动作名称、动作后台函数）
    FibonacciAction(std::string name) : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
                                        action_name_(name)
    {
        as_.start();
    }

    ~FibonacciAction(void)
    {
    }
    // 接收动作目标（goal）消息并执行指定动作（此处为斐波那契数列）的函数。
    void executeCB(const ros_action::TestGoalConstPtr &goal)
    {
        ros::Rate r(1);      // 循环周期：1 Hz
        bool success = true; // 用作保存动作的成功或失败的变量
        // 斐波那契数列的初始化设置，也添加了反馈的第一个（0）和第二个消息（1）
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);
        // 将动作名称、目标和斐波那契数列的两个初始值通知给用户
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
                 action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
        // 动作细节
        for (int i = 1; i <= goal->order; i++)
        {
            // 从动作客户端得知动作取消
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str()); // 通知动作取消
                as_.setPreempted();                              // 取消动作
                success = false;                                 // 看作动作失败并保存到变量
                break;
            }
            // 除非有动作取消或已达成动作目标
            // 将当前斐波纳契数字加上前一个数字的值保存到反馈值。
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
            as_.publishFeedback(feedback_); // 发布反馈。
            r.sleep();                      // 按照上面定义的循环周期调用暂歇函数。
        }
        // 如果达到动作目标值，则将当前斐波那契数列作为结果值传输。
        if (success)
        {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};
int main(int argc, char **argv) // 节点主函数
{
    ros::init(argc, argv, "action_server");           // 初始化节点名称
    FibonacciAction fibonacci("ros_action_test");     // 声明Fibonacci （动作名: ros_action_test）
    ros::spin();                                      // 等待动作目标
    return 0;
}