#!/usr/bin/env python3
import rospy
import time
from socket_moveTo import send_receive_data_moveTo, parse_response_moveTo
from std_msgs.msg import Float64MultiArray
import numpy as np

np.set_printoptions(suppress=True, precision=6)

#关节角度限位（单位：deg）
joint_limits = {
    1: (-180, 180),
    2: (-155, 5),
    3: (-20, 240),
    4: (-180, 180),
    5: (-95, 95),
    6: (-360, 360),
}

def check_limits(q_deg):
    """检查关节角度是否超限，并修正"""
    q_checked = q_deg.copy()
    within = True
    for i in range(6):
        q_min, q_max = joint_limits[i+1]

        if q_checked[i] < q_min:
            error = q_min - q_checked[i]
            q_checked[i] = q_min
            print(f"[限位警告] 关节 {i+1} 达到下限 {q_min}, 修正误差: {error:.6f}")
            within = False

        elif q_checked[i] > q_max:
            error = q_checked[i] - q_max
            q_checked[i] = q_max
            print(f"[限位警告] 关节 {i+1} 达到上限 {q_max}, 修正误差: {error:.6f}")
            within = False

    return within, q_checked


# 全局变量：从 Ruckig 订阅得到的关节角(rad)
ruckig_joints_pos_rad = None

def ruckig_pos_callback(msg):
    """订阅 Ruckig 输出关节角(rad)"""
    global ruckig_joints_pos_rad

    if len(msg.data) != 6:
        rospy.logerr("ruckig_joints_pos_rad 长度错误！")
        return

    ruckig_joints_pos_rad = np.array(msg.data, dtype=float)
    ruckig_joints_pos_deg = ruckig_joints_pos_rad*180/np.pi
    print(f"Ruckig计算得到的关节角度(rad)：{ruckig_joints_pos_rad }")
    print(f"Ruckig计算得到的关节角度(deg)：{ruckig_joints_pos_deg }")



#主循环：接收 Ruckig → 转成 deg → 限位检查 → socket发送
def get_joints_pos_with_ruckig():
    rospy.init_node("get_joints_pos_with_ruckig_node", anonymous=True)

    #订阅 Ruckig 输出
    rospy.Subscriber("/otg_joint_position_rad",
                     Float64MultiArray,
                     ruckig_pos_callback)

    # socket 连接参数
    server_ip = "10.10.56.214"
    server_port = 23333
    serial_number = 1

    # moveTo 固定参数
    gpId = 0
    isJoint = "true"
    ufNum = -1
    utNum = -1
    config = 0
    isLinear = "false"

    rate = rospy.Rate(100)     # 按 100Hz 循环

    while not rospy.is_shutdown():
        try:
            global ruckig_joints_pos_rad

            if ruckig_joints_pos_rad is None:
                rate.sleep()
                continue

            #ruckig_pos_rad → ruckig_pos_deg
            q_deg = ruckig_joints_pos_rad * 180.0 / np.pi

            #限位检查
            _, q_deg_limited = check_limits(q_deg)

            # 3.组装 moveTo 命令字符串发送
            new_strPos = "{" + ",".join(f"{x:.6f}" for x in q_deg_limited) + "}"

            command = (f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{new_strPos}",{isLinear})')

            response = send_receive_data_moveTo(server_ip, server_port, serial_number, command)
            Data = parse_response_moveTo(response)
            print(f"[机械臂返回] {Data}")

        except Exception as e:
            rospy.logerr(f"Error: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        get_joints_pos_with_ruckig()
    except rospy.ROSInterruptException:
        pass

