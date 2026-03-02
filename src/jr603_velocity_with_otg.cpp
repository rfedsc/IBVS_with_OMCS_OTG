#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <array>
#include <string>
#include "ruckig_velocity_min_duration.hpp"
#include <cmath> // 为了使用 M_PI

OTGManager otg(0.1); 

std::array<double,6> initial_position = {0,0,0,0,0,0};
std::array<double,6> initial_velocity = {0,0,0,0,0,0};
std::array<double,6> initial_acc = {0,0,0,0,0,0};

std::array<double,6> current_position = {0,0,0,0,0,0};
std::array<double,6> current_velocity = {0,0,0,0,0,0};
std::array<double,6> current_acc = {0,0,0,0,0,0};

std::array<double,6> target_position = {0,0,0,0,0,0};
std::array<double,6> target_velocity = {0,0,0,0,0,0};
std::array<double,6> target_acc = {0,0,0,0,0,0};

std::array<double,6> max_acc = {10,10,10,10,10,10};
std::array<double,6> max_jerk = {100,100,100,100,100,100};


// 状态标志位
bool has_new_target_vel = false;

ros::Publisher smooth_vel_pub;


std::array<double,6> last_target_vel = {0,0,0,0,0,0};

// 订阅/joints_velocity，更新目标速度
void jointsVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if(msg->data.size()!=6){
        ROS_ERROR("joints_velocity size!=6");
        return;
    }

    for(int i=0;i<6;i++){
        target_velocity[i] = msg->data[i];
    }
    
    ROS_INFO("Receive target joints_velocity without ruckig: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
             target_velocity[0],target_velocity[1],target_velocity[2],
             target_velocity[3],target_velocity[4],target_velocity[5]);
             
    has_new_target_vel = true;
    
    // 更新Ruckig目标速度
    //otg.update_velocity_acc(new_velocity,zero_acc);
    //ROS_INFO("Update Ruckig Velocity.");
}

int main(int argc,char** argv){

    ros::init(argc,argv,"otg_node");
    ros::NodeHandle nh;
    
    // 订阅/joints_velocity：队列大小10
    ros::Subscriber sub = nh.subscribe("/joints_velocity", 10, jointsVelocityCallback);
    // 发布平滑后的速度到/otg_velocity
    smooth_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/otg_velocity", 10);

    otg.initialize(
        current_position,
        current_velocity,
        current_acc,
        target_velocity,
        target_acc,
        max_acc,
        max_jerk,
        0.1); 
        
    ros::Rate loop_rate(10);
    ROS_INFO("OTG node start wait /joints_velocity...");

    while(ros::ok()){
        ros::spinOnce(); 
        
        otg.step();
        
        if ((has_new_target_vel==true)) {
            // 更新Ruckig目标速度
            
    	    //otg.update_velocity_acc(target_velocity,target_acc);
            ROS_INFO("Update Ruckig Velocity.");
            has_new_target_vel = false;
        }
        
        // 获取Ruckig输出的当前速度
        std::array<double,6> pos,vel,acc,jerk;
        
        otg.get_current_state(pos,vel,acc,jerk);

        // 发布平滑后的速度
        std_msgs::Float64MultiArray msg;
        msg.data.resize(6);
        for(int i=0;i<6;i++){
            //msg.data[i] = vel[i]; // 直接输出rad/s，不转换
            msg.data[i] = vel[i] * 180.0 / M_PI; // rad/s → deg/s
        }
        smooth_vel_pub.publish(msg);
        // 关键：打印输出的速度，确认是否非0
        ROS_INFO("[current otg_velocity: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f | target joints_velocity: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                 vel[0],vel[1],vel[2],vel[3],vel[4],vel[5],
                 target_velocity[0],target_velocity[1],target_velocity[2],
                 target_velocity [3],target_velocity[4],target_velocity[5]);

        loop_rate.sleep();    
    }
    return 0;
}
