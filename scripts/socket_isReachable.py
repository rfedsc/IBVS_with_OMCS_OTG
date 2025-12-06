#!/usr/bin/env python3
import re
import socket
def send_receive_data(server_ip,server_port,serial_number,command):
    """
    发送数据到服务器并接收响应
    :param server_ip:服务器的IP地址
    :param server_port:服务器的端口号
    :param serial_number:流水号(uint)
    :param command:终端命令
    :return: 服务器的响应
    """
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        #连接到服务器
        sock.connect((server_ip,server_port))
        #格式化发送的消息
        message = f"i:{serial_number},c:{command}@hs@"
        sock.sendall(message.encode('utf-8'))
        print(f"send_command:{message}")
        #接收服务器返回的消息
        response = sock.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error:{e}")
        return None
    finally:
        #关闭套接字
        sock.close()

def parse_response_getConfig(response):
    data = None
    if response:
        try:
            print(f"Raw response:{response}")
            match = re.search(r"d:(\d+)",response)
            if match:
                data = match.group(1)
                print(f"data is:{data}")
            else:
                print("No match found")
        except Exception as e:
            print(f"Error parsing response:{e}")
    else:
        print("No response")
    return data

    
def parse_response_moveTo(response):
    if response:
        print(f"Raw response:{response}")
        #这里根据响应格式解析
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed")
    else:
        print(f"No response")



def parse_response_isReachable(response):
    data = None
    if response:
        try:
            print(f"Raw response:{response}")
            match = re.search(r"d:(.*?)@h",response)
            if match:
                data = match.group(1)
                print(f"data is:{data}")
            else:
                print("No match found")
        except Exception as e:
            print(f"Error parsing response:{e}")
    else:
        print("No response")
    return data



#使用示例
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1

    #用户输入参数
    gpId = 0
    isJoint = 'true'
    ufNum = -1
    utNum = -1
    config = None
    strPos = '"{50,0,1085.5,180.0,0,180,0,0,0}"'
    isLinear = 'false'    

    command_getConfig = f"mot.getConfig({gpId})"

    #发送数据并接收响应
    response = send_receive_data(server_ip,server_port,serial_number,command_getConfig)
    #解析收到的响应
    config = parse_response_getConfig(response)

    type = '1'    
    command_isReachable = f"mot.isReachable({gpId},{isJoint},{ufNum},{utNum},{1048576},{strPos},{type})"
    #发送数据并接收响应
    response = send_receive_data(server_ip,server_port,serial_number,command_isReachable)
    #解析收到的响应
    flag = parse_response_isReachable(response)
    print(f"flag is:{flag}")

    if flag == 'true':
        print("点位可达")
        command_moveTo = f"mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},{strPos},{isLinear})"
        #发送数据并接收响应
        response = send_receive_data(server_ip,server_port,serial_number,command_moveTo)
        #解析收到的响应
        Data = parse_response_moveTo(response)
        print(f"Data is:{Data}")
    else:
        print(f"点位不可达,请调整角度或运行方式")


