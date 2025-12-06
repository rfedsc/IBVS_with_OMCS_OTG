#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from lys_visp_demo.srv import SendCommand
import socket

def send_receive_data(server_ip,server_port,serial_number,command):
    """
    发送数据到服务器并接收响应
    """
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        #连接到服务器
        sock.connect((server_ip,server_port))
        #格式化发送的消息
        message = f"i:{serial_number},c:{command}@hs@"
        sock.sendall(message.encode('utf-8'))

        #接收服务器返回的消息
        response = sock.recv(1024).decode('utf-8')
        return response

    except socket.error as e:
        rospy.logerr(f"Socket error:{e}")
        return None
    finally:
        #关闭套接字
        sock.close()

def parse_response(response):
    """
    解析接收到的服务器响应
    """
    data_dict = {}
    if response:
        try:
            response = response.split('@hs@')[0]
            parts = response.split(',')
            for part in parts:
                if ':' in part:
                    key,value = part.split(':',1)    
                    data_dict[key.strip()] = value.strip()
                else:
                    rospy.logwarn(f"Unexpect part format:{part}")
        except Exception as e:
            rospy.logerr(f"Error parsing response:{e}")

    return data_dict

def handle_send_command(req):
    """
    ROS服务回调函数:处理发送命令请求
    """
    server_ip = req.server_ip
    server_port = req.server_port
    serial_number = req.serial_number
    command = req.command
    
    #发送数据并接收响应
    response = send_receive_data(server_ip,server_port,serial_number,command)
    parse_data = parse_response(response)
    #将字典格式的解析数据转换为字符串以便返回
    parsed_data_str = str(parse_data)
    return parsed_data_str

if __name__ == '__main__':
    rospy.init_node('socket_client_node')
    rospy.Service('send_command',SendCommand,handle_send_command)
    rospy.loginfo("Socket client service is ready")
    rospy.spin()


