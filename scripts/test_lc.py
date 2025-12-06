import time

# 从你已有的代码导入函数（或粘贴函数定义）
from socket_moveTo import send_receive_data_moveTo, parse_response_moveTo

# === 1. 读取 data.txt 文件 ===
def load_pose_data(filepath):
    pose_list = []
    with open(filepath, 'r') as f:
        for line in f:
            # 支持空格或逗号分隔
            line = line.strip().replace(',', ' ')
            if not line:
                continue
            values = list(map(float, line.split()))
            if len(values) != 6:
                raise ValueError(f"Invalid pose line: {line}")
            pose_list.append(values)
    return pose_list

# === 2. 参数设置 ===
server_ip = '10.10.56.214'
server_port = 23333
serial_number = 1
gpId = 0
isJoint = 'true'
ufNum = -1
utNum = -1
config = 0
isLinear = 'false'

# === 3. 主程序 ===
if __name__ == "__main__":
    pose_data = load_pose_data("angle2.txt")
    print(f"✅ 读取到 {len(pose_data)} 组位姿数据，开始发送...\n")

    for i, pose in enumerate(pose_data):
        strPos = '"{' + ','.join(f"{v:.7f}" for v in pose) + '}"'
        command = f"mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},{strPos},{isLinear})"

        print(f"Sending command {i + 1}/{len(pose_data)}: {command}")
        response = send_receive_data_moveTo(server_ip, server_port, serial_number, command)
        parse_response_moveTo(response)

        serial_number += 1
        time.sleep(0.5)  # 可根据执行耗时调节

    print("\n✅ 所有命令发送完毕。")

