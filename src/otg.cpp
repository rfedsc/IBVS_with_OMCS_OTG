#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <array>
#include <string>
#include "ruckig_velocity_min_duration.hpp"

OTGManager otg(0.01); //控制周期1ms

std::array<double,6> current_position = {0,0,0,0,0,0};
std::array<double,6> current_velocity = {0,0,0,0,0,0};
std::array<double,6> current_acc = {0,0,0,0,0,0};

std::array<double,6> max_acc = {1,1,1,1,1,1};
std::array<double,6> max_jerk = {5,5,5,5,5,5};

ros::Publisher smooth_vel_pub;

//订阅joints_velocity
void jointsVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if(msg->data.size()!=6){
        ROS_ERROR("joints_velocity size!=6");
	return;
    }

    std::array<double,6> new_velocity;
    std::array<double,6> zero_acc = {0,0,0,0,,0,0};

    for(int i=0;i<6;i++){
    	new_velocity[i] = msg->data[i];
    }
    //更新Ruckig目标速度
    otg.update_velocity_acc(new_velocity,zero,acc);
}

int main(int argc,char** argv){

    ros::init(argc,argv,"otg_node");
    ros::NodeHandle nh;
    //订阅关节速度
    ros::Subssscriber sub = nh.subscribe("/joints_velocity",10,jointsVelocityCallback);
    //发布平滑后的速度
    otg_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/otg_velocity",10)

    //设置输出周期
    ros::Rate loop_rate(1000); //1KHz
    //初始化Ruckig
    otg.initiallize(
	current_position,
	current_velocity,
	current_acc,
	{0,0,0,0,0,0},//目标速度
	{0,0,0,0,0,0},//目标加速度
	max_acc,
	max_jerk,
	0,0);
    while(ros::ok()){
        ros::spinOnce();
	//Ruckig单步更新
	otg.step();
	//获取输出状态
	std::array<double,6> pos,vel,acc,jerk;
	otg.get_current_state(pos,vel,acc,jerk);
	//发布平滑后的关节速度
	std_msgs::Float64MultiArray msg;
	msg.data = {vel[0],vel[1],vel[2],vel[3],vel[4],vel[5]};
        otg_vel_pub.publish(msg);
        loop_rate.sleep();	
    }
}


