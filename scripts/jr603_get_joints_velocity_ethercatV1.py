#!/usr/bin/env python3
import rospy
import time
from jacob_cross_SDH import jacob_cross_sdh
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerRequest
import numpy as np



def get_end_velocity():
    """获取末端速度"""
    end_v_msg = rospy.wait_for_message('/end_velocity', Twist)
    end_v = np.array([
        end_v_msg.linear.x,
        end_v_msg.linear.y,
        end_v_msg.linear.z,
        end_v_msg.angular.x,
        end_v_msg.angular.y,
        end_v_msg.angular.z
    ])
    print(f"获取到的末端速度:{end_v}")
    return end_v
    
    
#通过ROS服务获取关节角度
def get_joints_angle_from_service():
    """调用驱动节点的ROS服务，获取6维关节角度（deg）"""
    try:
        # 等待服务可用
        rospy.wait_for_service('/get_joints_angle')
        # 创建服务代理
        get_joints_proxy = rospy.ServiceProxy('/get_joints_angle', Trigger)
        # 调用服务
        resp = get_joints_proxy(TriggerRequest())
        if resp.success:
            angle_list = [float(angle_str) for angle_str in resp.message.split(";")]
            angles = {
                'q1': angle_list[0], 'q2': angle_list[1], 'q3': angle_list[2],
                'q4': angle_list[3], 'q5': angle_list[4], 'q6': angle_list[5]
            }
            return angles
        else:
            rospy.logerr(f"获取关节角度失败：{resp.message}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"调用ROS服务失败：{e}")
        return None
    except (ValueError, IndexError) as e:
        rospy.logerr(f"解析关节角度失败：{e}")
        return None

def get_joints_velocity():  
    # 初始化ROS节点
    rospy.init_node('get_joints_velocity_node', anonymous=True)
    # 创建 joints_velocity 发布者
    joints_vel_pub = rospy.Publisher(
        '/joints_velocity',
        Float64MultiArray,
        queue_size=10
    )
    rate = rospy.Rate(10)  # 10Hz
    # --------------------------------------------------------------------------------
    while not rospy.is_shutdown():
        try:
        
            angles = get_joints_angle_from_service()
            # 以下业务逻辑完全不变
            angle_list = [angles['q1'],angles['q2'],angles['q3'],angles['q4'],angles['q5'],angles['q6']]
            # 转换为弧度
            q = np.deg2rad(angle_list)
            print(f"当前角度(弧度):\n{q}")
            
            # 计算雅可比矩阵
            J = jacob_cross_sdh(q)
            print(f"当前角度(弧度)下的雅可比矩阵:\n{J}")
            
            # 获取末端速度
            end_velocity = get_end_velocity()
            
            # 求关节速度（伪逆处理奇异性）
            try:
                q_velocity = np.linalg.inv(J).dot(end_velocity)
            except np.linalg.LinAlgError:
                q_velocity = np.linalg.pinv(J).dot(end_velocity)
                
            q_velocity =  q_velocity*180/np.pi;   
            
            print(f"计算出的关节速度:\n{q_velocity}")
            
            # 发布关节速度
            msg = Float64MultiArray()
            msg.data = q_velocity.tolist()
            joints_vel_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"循环执行错误: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        get_joints_velocity()
    except rospy.ROSInterruptException:
        pass
