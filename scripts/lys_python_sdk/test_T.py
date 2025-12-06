import numpy as np
from ikine_jr603 import forward_kinematics,inverse_kinematics
from socket_getJntData import send_receive_data,parse_response
from socket_moveTo import send_receive_data_moveTo,parse_response_moveTo
from dh import a,d,alp,offset,eMc


def test():
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1
    command = "mot.getJntData(0)"
    #发送数据并接收响应
    response = send_receive_data(server_ip,server_port,serial_number,command)
    #解析收到的响应
    Data = parse_response(response)
    q = Data[0:6]
    T06 = forward_kinematics(q)
    #AprilTag标签相对于相机的齐次变换矩阵
    T_apriltag= np.array([[0.05794754, 0.65447246, -0.75386198, -0.49003842],
                  [-0.96309688, -0.16216641, -0.21481724, -0.16910298],
                  [-0.26284306, 0.73849025, 0.62092324, 1.20049342],
                  [0.0, 0.0, 0.0, 1.0]])
    #eMc为相机外参,即相机相对于机械臂末端的齐次变换矩阵
    T_object = T_apriltag@eMc@T06
    q_result = inverse_kinematics(T_object)
    
    #选择所有有效的解
    valid_solution = []
    for q_sol in q_result:
        #检查所有值是否均不为-1
        if np.all(q_sol != -1):
            valid_solution.append(q_sol)
    valid_solution = np.array(valid_solution)
    if valid_solution.size > 0:
        #选择一组有效的解
        q_1 = valid_solution[0]
        #格式化为字符串
        solution_str = "{"+",".join(f"{theta:.6f}" for theta in q_1)+"}"
        print(f"Selection solution:{solution_str}")
    else:
        print("No valid solution found")


    print(f"Data is:{Data}")
    print(f"q is:{q}")
    print(f"T06 is:\n {T06}")
    print(f"T_object is:\n{T_object}")
    print(f"q_result is:\n{q_result}")
    if valid_solution is not None:
        print(f"valid solution is:\n {valid_solution}")
    else:
        print("No valid solution found.")

    
    #给机械臂发送关节角度
    gpId = 0
    isJoint = 'true'
    ufNum = -1
    utNum = -1
    config = 0
    strPos = solution_str
    isLinear = 'false'
    command = f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{strPos}",{isLinear})'
    #发送数据并接收响应
    response = send_receive_data_moveTo(server_ip,server_port,serial_number,command)
    #解析收到的响应
    Data = parse_response_moveTo(response)


if __name__ == '__main__':
    test()

'''
#矩阵相乘
result = T06

q = inverse_kinematics(result)

#打印结果
print(f"Result of oMc*eMc*T06:\n {result}")
print(f"q is:\n {q}")
'''




