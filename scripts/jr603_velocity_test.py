#!/usr/bin/env python3
import rospy
import time
from jacob_cross_SDH import jacob_cross_sdh
from socket_isReachable import send_receive_data,parse_response_getConfig,parse_response_moveTo,parse_response_isReachable
from socket_getLocData import parse_response_getLocData
from geometry_msgs.msg import Twist
from pose2T import euler_to_rotation_matrix,cartesian_to_homogeneous
from ikine_jr603 import inverse_kinematics
 
import numpy as np

def get_end_velocity():
    end_v_msg = rospy.wait_for_message('/end_velocity',Twist)
    #获取末端速度
    end_v = np.array([end_v_msg.linear.x,
                      end_v_msg.linear.y,
                      end_v_msg.linear.z,
                      end_v_msg.angular.x,
                      end_v_msg.angular.y,
                      end_v_msg.angular.z])

    print(f"获取到的末端速度:{end_v}")
    return end_v
    
def test_velocity():  
 
        #初始化ROS节点
        rospy.init_node('test_velocity',anonymous=True)
        #每秒10次,即0.1秒控制周期
        rate = rospy.Rate(10)
        time_step = 0.1
        while not rospy.is_shutdown():
            try:
                server_port = 23333
                serial_number = 1
                server_ip = '10.10.56.214'        
       
                #用户输入参数
                gpId = 0
                isJoint = 'false'
                ufNum = -1
                utNum = -1
                config = None
                strPos = '"{287.502,-0.001,232.240,180.000,-0.000,180.000}"'
                isLinear = 'false'

                print(f"config is:{config}")
                #获取笛卡尔坐标
                command_getLocData = "mot.getLocData(0)"
                response = send_receive_data(server_ip,server_port,serial_number,command_getLocData)
                #解析笛卡尔坐标
                LocData = parse_response_getLocData(response)
                print(f"LocData is:{LocData}")
              
                #获取形态位
                command_getConfig = f"mot.getConfig({gpId})"
                response = send_receive_data(server_ip,server_port,serial_number,command_getConfig)
                #解析形态位
                config = parse_response_getConfig(response)
                print(f"config is:{config}")


                #获取末端速度
                end_velocity = get_end_velocity()
                print(f"end_velocity is:{end_velocity}")
        
                #将m/s转换为mm/s
                tx_v = end_velocity[0]*1000
                ty_v = end_velocity[1]*1000
                tz_v = end_velocity[2]*1000
                #rad/s转换为deg/s
                rx_v = end_velocity[3]*(180/np.pi)
                ry_v = end_velocity[4]*(180/np.pi)
                rz_v = end_velocity[5]*(180/np.pi)

                new_velocity = np.array([tx_v,ty_v,tz_v,rx_v,ry_v,rz_v])
                print(f"new_velocity[rx_v,ty_v,tz_v,rx_v,ry_v,rz_v] is:{new_velocity}")

                delta_tx = tx_v*time_step
                delta_ty = ty_v*time_step
                delta_tz = tz_v*time_step
                
                delta_rx = rx_v*time_step
                delta_ry = ry_v*time_step
                delta_rz = rz_v*time_step
                delta = np.array([delta_tx,delta_ty,delta_tz,delta_rx,delta_ry,delta_rz])
                print(f"delta[delta_tx,delta_ty,delta_tz,delta_rx,delta_ry,delta_rz]:{delta}")
                
                new_LocData = [LocData[0]+delta_tx,LocData[1]+delta_ty,LocData[2]+delta_tz,LocData[3]+delta_rz,LocData[4]+delta_ry,LocData[5]+delta_rx]
                print(f"new_LocData is:{new_LocData}")
                new_strPos = "{" + ",".join(map(lambda x: f"{x:.6f}", new_LocData)) + ",}"
                command_moveTo = f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{new_strPos}",{isLinear})'
                response = send_receive_data(server_ip,server_port,serial_number,command_moveTo)
                #解析收到的响应
                Data = parse_response_isReachable(response)
                print(f"Data is:{Data}")
                '''
                x = new_LocData[0]
                y=new_LocData[1]
                z=new_LocData[2]
                roll = new_LocData[5]
                pitch=new_LocData[4]
                yaw=new_LocData[3]
                T = cartesian_to_homogeneous(x,y,z,roll,pitch,yaw)
                angles = inverse_kinematics(T)
                valid_solutions = []
                for angle in angles:
                    if np.all(angle!=-1):
                        valid_solutions.append(angle)
                if len(valid_solutions)==0:
                    print("All solutions are invalid,skipping...")
                else:
                    #选择一组有效的解
                    q_1 = valid_solution[0]
                    #格式化为字符串
                    solution_str = "{"+",".joint(f"{theta:6f}" for theta in q_1) +"}"
                    print(f"Selected solution:{solution_str}")
                    new_strPos = solution_str
                    command = f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{new_strPos}",{isLinear})'
                    print(f"Comamnd to send:{command}")
                    response = send_receive_data_moveTo(server_ip, server_port, serial_number, command)
                    Data = parse_response_moveTo(response)
                '''
            except Exception as e:
                rospy.logerr(f"Error in test_velocity loop:{e}")
            
            #按定义的频率等待
            rate.sleep()

if __name__ == '__main__':
    try:
        test_velocity()
    except rospy.ROSInterruptException:
        pass




 
