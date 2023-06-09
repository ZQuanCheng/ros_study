// This is the C++ version server file of the service demo

// 加载必要文件，注意Service_demo的加载方式
# include "ros/ros.h"
# include "service_demo/Greeting.h"
# include "string"

// 定义请求处理函数
bool handle_function(service_demo::Greeting::Request &req,
					service_demo::Greeting::Response &res)
{
	//显示请求信息
	ROS_INFO("Request from %s with age %d ", req.name.c_str(), req.age);
	
	// 处理请求，结果写入response
	res.feedback = "Hi " + req.name + ". I'm server!";

    //返回true，正确处理了请求
	return true;
}

int main(int argc, char **argv)
{
	// 初始化节点，命名为"greetings_server"
	ros::init(argc, argv, "greetings_server");
	
	// 定义service的server端，service名称为“greetings”，收到request请求之后传递给handle_function进行处理
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("greetings", handle_function);
	
	ros::spin();

	return 0;
}
