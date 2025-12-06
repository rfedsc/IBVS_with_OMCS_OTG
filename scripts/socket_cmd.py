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

        #接收服务器返回的消息
        response = sock.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error:{e}")
        return None
    finally:
        #关闭套接字
        sock.close()
def parse_response(response):
    """
    解析接收到的服务器响应
    :param response:服务器的响应
    :return:解析后的数据字典
    """
    data_dict = {}
    if response:
        try:
            #假设响应的格式为“i:流水号,e:错误码,d:命令返回数据内容@hs@”
            #先分割分割符
            #仅取第一个分割符前的部分
            response = response.split('@hs@')[0]
            #以空格分割
            parts = response.split(',')
            for part in parts:
                if ':' in part:
                    #只分割第一个冒号
                    key,value = part.split(':',1)
                    data_dict[key.strip()] = value.strip()
                else:
                    #处理格式异常的部分
                    print(f"Unexpect part format:{part}")
        except Exception as e:
            print(f"Error parsing response:{e}")
    return data_dict
#使用示例
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1
    command = "mot.setGpEn(0,true)"

    #发送数据并接收响应
    response = send_receive_data(server_ip,server_port,serial_number,command)
    #解析收到的响应
    parsed_data = parse_response(response)

    print("Parsed data:",parsed_data)


