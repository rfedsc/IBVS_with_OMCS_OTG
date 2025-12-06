from pymycobot.mycobot import MyCobot
import time
from mycobot_init import MyCobot_Init

mycobot = MyCobot_Init()

#设置初始位置
start_angles = mycobot.get_coords()
mycobot.send_angles(start_angles,30)

#等待机械臂移动到初始位置
time.sleep(2)

#定义目标位置
target_angles = [30,45,60,30,45,60]
#T型速度控制
def t_velocity_control(target_angles,max_speed,accel_time,decel_time):
  #加速段
  for speed in range(10,max_speed,10):
    mycobot.send_angles(target_angles,speed)
    time.sleep(accel_time)
  
  #恒速段
  mycobot.send_angles(target_angles,max_speed)
  time.sleep(2)
  
  #减速段
  for speed in range(max_speed,10,-10):
    mycobot.send_angles(target_angles,speed)
    time.sleep(decel_time)

#调用T型速度控制
t_velocity_control(target_angles,max_speed=70,accel_time=0.5,decel_time=0.5)

#确保机械臂达到目标位置
mycobot.send_angles(target_angles,30)






