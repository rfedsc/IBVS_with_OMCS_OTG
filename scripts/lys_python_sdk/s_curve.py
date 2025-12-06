from pymycobot import utils
import numpy as np
import time
from pymycobot import MyCobot
from mycobot_init import MyCobot_Init

mycobot = MyCobot_Init()

#定义起始位置
start_position = mycobot.get_coords()
target_position = [30,45,60,45,30,15]

#运动时间和步长
T = 2.0 #总时间(秒)
dt = 0.1#时间步长(秒)
t = np.arange(0,T,dt)

#计算S型速度曲线
def s_curve(t,T):
  a_max = 2*100/T #最大加速度(假设)
  v_max = 100/T #最大速度(假设)
  tau = v_max/a_max#计算速度达到最大值需要的时间
  if t<tau: #在加速阶段
    return 0.5*a_max*t**2 #返回位移
  elif t<T-tau: #匀速运动阶段
    return 0.5*a_max*tau**2 + v_max*(t-tau)
  else:#减速阶段
    t_d = t-(T-tau)
    return 0.5*a_max*tau**2+v_max*(T-2*tau)+v_max*t_d-0.5*a_max*t_d**2

def s_curve_velocity(t,T):
  a_max = 2*100/T #最大加速度(假设)
  v_max = 100/T #最大速度(假设)
  tau = v_max/a_max #计算速度达到最大数值需要的时间
  if t<tau: #在加速阶段
    return a_max*t #返回速度
  elif t<T-tau: #匀速运动阶段
    return v_max
  else: #减速阶段
    t_d = t-(T-tau)
    return v_max-a_max*t_d #返回速度

def s_curve_acceleration(t,T):
  a_max = 2*100/T #最大加速度(假设)
  v_max = 100 #最大速度(假设)
  tau = v_max/a_max #计算速度达到最大值需要的时间
  if t<tau:#在加速阶段
    return a_max #返回加速度
  elif t<T-tau: #匀速运动阶段
    return 0 #返回加速度
  else:#减速阶段
    return -a_max #返回加速度


#计算总位移动
total_displacement = s_curve(T,T)

#生成每个时间步的目标位置
for time_step in t:
  progress = s_curve(time_step,T)/total_displacement #计算当前时间步time_step在s型曲线下的位移
  #计算current_position
  current_position =  [(1-progress)*start+progress*target
                          for start,target in zip(start_position,target_position)]

  #计算速度和加速度
  velocity = s_curve_velocity(time_step,T)
  acceleration = s_curve_acceleration(time_step,T)
  #动态设置速度参数
  speed_param =int( max(10,min(100,velocity))) #根据需要调整速度范围

  #打印速度和加速度
  print(f"Time:{time_step:.1f}s,Velocity:{velocity:.2f} units/s Acceleration:{acceleration:.2f} units/s²,Speed Param:{speed_param:.2f}")
  
  
  #发送命令到机械臂
  mycobot.send_angles(current_position,speed_param)
  #获取规划速度和加速度
#  for i in range(10):
  speeds=mycobot.get_plan_speed()
  accelerations=mycobot.get_plan_acceleration()
  time.sleep(dt)
# 打印每个关节的速度和加速度
  for i, (speed, accel) in enumerate(zip(speeds, accelerations)):
     print(f"Joint {i}: Speed={speed:.2f} units/s, Acceleration={accel:.2f} units/s²")
  v=mycobot.get_speed()
 # print(f"规划速度:{s}")
 # print(f"规划加速度:{a}")
  print(f"速度:{v}")
  

  #等待时间步长
 # time.sleep(dt)

#mycobot.send_angles(target_position,50)



