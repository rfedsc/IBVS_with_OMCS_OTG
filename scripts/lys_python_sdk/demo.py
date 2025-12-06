from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT,PI_BAUD
import time

def initialize_mycobot(port=PI_PORT,baudrate=PI_BAUD):
  #初始化MyCobot机械臂
  mycobot = MyCobot(port,baudrate)
  mycobot.power_on()
  return mycobot

def generate_linear_trajectory(start,end,steps):
  #生成笛卡尔空间的直线轨迹
  trajectory = []
  for i in range(steps):
    point = [start[j] + (end[j] - start[j]) * i / (steps - 1) for j in range(len(start))]
    trajectory.append(point)
    formatted_point = ["{:.2f}".format(coord) for coord in point]
    print("Step {}:{}".format(i,formatted_point))
  return trajectory


def execute_trajectory(mycobot,trajectory,speed):
  #逐渐步发送轨迹点到MyCobot进行执行
  for point in trajectory:
    mycobot.send_coords(point,speed,mode=1) #mode=1表示直线运动
    time.sleep(0.1) #确保每个指令有足够的时间执行

def main():
  # 初始化机械臂
  mycobot = initialize_mycobot()
  mycobot.send_coords([0,0,0,0,0,0],50,1)
  time.sleep(10)
  #获取当前的位姿
  start_coords = mycobot.get_coords()
  print("start coords:",start_coords)
  
  #定义终点位姿
  end_coords = [50,50,50,0,0,0]
  
  #生成轨迹
  steps = 50 #插值点的数量
  speed = 50 #运动速度
  trajectory = generate_linear_trajectory(start_coords,end_coords,steps)
  #执行轨迹
  execute_trajectory(mycobot,trajectory,speed)
  end_coords = mycobot.get_coords()
  print("end coords:",end_coords)

  print("Trajectory execution completed.")
 # mycobot.release_all_servos()
 # mycobot.power_off()

if __name__ == "__main__":
 main()   























