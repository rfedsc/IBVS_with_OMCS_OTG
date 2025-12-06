#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <array>
#include <iostream>
#include <cmath> 
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ruckig_pos_mode.hpp" 

// 全局OTG管理器
OTGManager otg(0.01);  

// 全局状态变量
std::array<double, 6> init_curr_pos   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
std::array<double, 6> init_curr_vel   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
std::array<double,6>  init_curr_acc =   {0.0,0.0,0.0,0.0,0.0,0.0};

std::array<double, 6> init_target_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
std::array<double, 6> init_target_vel   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::array<double,6>  init_target_acc = {0.0,0.0,0.0,0.0,0.0,0.0};

std::array<double, 6> curr_pos   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //初始化为0,可从ROS话题获取
std::array<double, 6> curr_vel   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //初始化为0,可从ROS话题获取
std::array<double,6>  curr_acc =   {0.0,0.0,0.0,0.0,0.0,0.0};

std::array<double, 6> target_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //初始化为0,可从ROS话题获取
std::array<double, 6> target_vel   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //初始化为0,可从ROS话题获取
std::array<double,6> target_acc = {0.0,0.0,0.0,0.0,0.0,0.0};

std::array<double,6> max_vel  = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
std::array<double,6> max_acc  = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
std::array<double,6> max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

// 状态标志位
bool has_curr_pos = false;
bool has_curr_vel = false;
bool has_target_pos = false; 
bool has_target_vel = false;

bool task_finished = false;

// ROS发布器
ros::Publisher pub_otg_pos;

// 当前关节角度回调 (rad)
void currPosCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 6) {
        ROS_WARN("Invalid curr_pos data size! Expected 6, got %lu", msg->data.size());
        return;
    }

    for (int i = 0; i < 6; i++) {
        curr_pos[i] = msg->data[i];
    }

    has_curr_pos = true;
    ROS_INFO("Current joint angle obtained (rad): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             curr_pos[0], curr_pos[1], curr_pos[2],
             curr_pos[3], curr_pos[4], curr_pos[5]);
    ROS_INFO("Current joint angle obtained (deg): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             curr_pos[0] * 180 / M_PI, curr_pos[1] * 180 / M_PI,
             curr_pos[2] * 180 / M_PI, curr_pos[3] * 180 / M_PI,
             curr_pos[4] * 180 / M_PI, curr_pos[5] * 180 / M_PI);
}

// 当前关节速度回调 (rad/s)
void currVelCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 6) {
        ROS_WARN("Invalid last_vel data size! Expected 6, got %lu", msg->data.size());
        return;
    }

    for (int i = 0; i < 6; i++) {
        curr_vel[i] = msg->data[i];
    }

    has_curr_vel = true;
    ROS_INFO("Current joint velocity obtained (rad/s): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             curr_vel[0], curr_vel[1], curr_vel[2],
             curr_vel[3], curr_vel[4], curr_vel[5]);
}

// 目标关节速度回调 (rad/s)
void targetVelCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 6) {
        ROS_WARN("Invalid next_vel data size! Expected 6, got %lu", msg->data.size());
        return;
    }

    for (int i = 0; i < 6; i++) {
        target_vel[i] = msg->data[i];
    }

    has_target_vel = true;
    ROS_INFO("Target joint velocity obtained (rad/s): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             target_vel[0], target_vel[1], target_vel[2],
             target_vel[3], target_vel[4], target_vel[5]);
}

// 目标关节角度回调
void targetPosCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 6) {
        ROS_WARN("Invalid target_pos data size! Expected 6, got %lu", msg->data.size());
        return;
    }

    // 更新目标位置
    for (int i = 0; i < 6; i++) {
        target_pos[i] = msg->data[i];
    }

    has_target_pos = true; // 确保主循环持续执行step
    ROS_INFO("=== New target received! Switch immediately ===");
    ROS_INFO("New target joint angle (rad): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             target_pos[0], target_pos[1], target_pos[2],
             target_pos[3], target_pos[4], target_pos[5]);
    ROS_INFO("New target joint angle (deg): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             target_pos[0] * 180 / M_PI, target_pos[1] * 180 / M_PI,
             target_pos[2] * 180 / M_PI, target_pos[3] * 180 / M_PI,
             target_pos[4] * 180 / M_PI, target_pos[5] * 180 / M_PI);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "jr603_pos_with_otg");
    ros::NodeHandle nh;

    // 订阅话题（队列大小设为1，确保收到最新目标）
    ros::Subscriber sub_curr_pos = nh.subscribe("/joint_curr_position_rad", 1, currPosCallback);
    ros::Subscriber sub_last_vel = nh.subscribe("/joint_last_velocity_rad_s", 1, currVelCallback);
    ros::Subscriber sub_target_pos = nh.subscribe("/joint_target_position_rad", 1, targetPosCallback);
    ros::Subscriber sub_next_vel = nh.subscribe("/joint_next_velocity_rad_s", 1, targetVelCallback);
    
    // 发布Ruckig计算后的关节角度
    pub_otg_pos = nh.advertise<std_msgs::Float64MultiArray>("/otg_joint_position_rad", 10);
    
    // 等待首次当前关节角度数据（避免初始化用默认值）
    ROS_INFO("Waiting for initial current joint position...");
    while (!has_curr_pos && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    ROS_INFO("Initial current joint position received!");
    
    otg.initialize(curr_pos, init_curr_vel, init_curr_acc,curr_pos, init_target_vel, init_target_acc,max_vel, max_acc, max_jerk);
    
    // 设置循环频率（100Hz，与control_dt=0.01s匹配）
    ros::Rate rate(100);
    // 主循环（持续执行，收到新目标即刻切换）
    while (ros::ok()) {
        ros::spinOnce();
        // 执行Ruckig单步计算
        otg.step();
        
        //如果有新的target_pos
        //if ((has_curr_pos==true)&&(has_target_pos==true)&&(has_curr_vel==true)&&(has_target_vel==true)) {
        if ((has_curr_pos==true)&&(has_target_pos==true)) {
            // 更新OTG管理器
            //otg.update_targets(curr_pos, curr_vel, curr_acc,target_pos, target_vel, target_acc,max_vel, max_acc, max_jerk);
            //otg.update_targets(curr_pos, init_curr_vel, init_curr_acc,target_pos, init_target_vel, init_target_acc,max_vel, max_acc, max_jerk);
            otg.update_targets(curr_pos, curr_vel, init_curr_acc,target_pos, init_target_vel, target_acc,max_vel, max_acc, max_jerk);
            has_curr_pos=false;
            has_target_pos=false;
        }
            
        // 读取当前计算状态（位置/速度/加速度/加加速度）
        std::array<double,6> pos, vel, acc, jerk;
        otg.get_current_state(pos, vel, acc, jerk);
        
	//打印 Ruckig 输出的 pos
	ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
	ROS_INFO("[OTG pos update] (rad): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
         	pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

	ROS_INFO("[OTG pos update] (deg): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
         		          pos[0] * 180.0 / M_PI, pos[1] * 180.0 / M_PI,
         		    	  pos[2] * 180.0 / M_PI, pos[3] * 180.0 / M_PI,
         		          pos[4] * 180.0 / M_PI, pos[5] * 180.0 / M_PI);
        ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        // 发布计算后的关节角度
        std_msgs::Float64MultiArray msg;
        msg.data.resize(6);
        for (int i = 0; i < 6; i++) {
            msg.data[i] = pos[i];
        }

        // 打印调试信息（突出当前目标和计算结果）
        ROS_INFO("----------------------------------------");
        ROS_INFO("Current target position (rad): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 target_pos[0], target_pos[1], target_pos[2],
                 target_pos[3], target_pos[4], target_pos[5]);
        ROS_INFO("Current target position (deg): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 target_pos[0] * 180 / M_PI, target_pos[1] * 180 / M_PI, target_pos[2] * 180 / M_PI,
                 target_pos[3] * 180 / M_PI, target_pos[4] * 180 / M_PI, target_pos[5] * 180 / M_PI);
        ROS_INFO("Ruckig planned position (rad): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
        ROS_INFO("Ruckig planned position (deg): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 pos[0] * 180 / M_PI, pos[1] * 180 / M_PI, pos[2] * 180 / M_PI,
                 pos[3] * 180 / M_PI, pos[4] * 180 / M_PI, pos[5] * 180 / M_PI);
        ROS_INFO("Ruckig planned velocity (rad/s): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
        ROS_INFO("----------------------------------------");

        // 发布话题
        pub_otg_pos.publish(msg);


        rate.sleep();
    }

    ROS_INFO("Node exit normally.");
    return 0;
}
