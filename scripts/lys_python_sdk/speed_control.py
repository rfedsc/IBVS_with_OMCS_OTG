import numpy as np
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT,PI_BAUD
from jacob_cross_SDH import jacob_cross_sdh
from mycobot_init import MyCobot_Init,joints_limit
import time

 
#期望末端速度(线速度:vx,vy,vz;角速度:wx,wy,wz)
T_velocity = np.array([0.5,0.5,0.5,0.5,0.5,0.5])
print(f"期望的末端速度:{T_velocity}")
#期望的末端位姿(单位弧度)
target_degree = np.array([-1.5,-1.5,2,-2,2,1])
print(f"期望的末端位置:{target_degree}")

#当前关节角度
mycobot = MyCobot_Init()
q = mycobot.get_radians()
#print("当前的关节角度:",q)

#计算雅可比矩阵
#J = jacob_cross_sdh(q)
#计算雅可比矩阵的伪逆
#J_pinv = np.linalg.pinv(J)
#通过雅可比矩阵的伪逆计算关节角速度
#joint_velocity = np.dot(J_pinv,T_velocity)



#控制机械臂逐步达到目标位置并确保关节速度匹配
#时间步长(s)
step_time = 0.1
#控制持续时间
duration = 10.0
#控制的步数
steps = int(duration/step_time)

for step in range(steps):
  q = None
  #循环直到成功获取有效的关节角度
  while q is None or len(q) != 6: 
    #获取当前的关节角度
    q = mycobot.get_radians()
   #print(f"当前的实际关节角度:{q}")
    if len(q) != 6:
      print(f"Step{step + 1}:错误,无法获取有效的关节角度,当前返回:{q}")
      #等待一段时间后重试
      time.sleep(step_time)

  #计算雅可比矩阵
  J = jacob_cross_sdh(q)
  #计算雅可比矩阵的伪逆
  J_pinv = np.linalg.pinv(J)
  #通过雅可比矩阵的伪逆计算关节角速度
  joint_velocity = np.dot(J_pinv,T_velocity)
  print(f"雅可比矩阵计算出的关节角速度:{joint_velocity}")

  #计算当前位置到目标位置的差距
  position_error = target_degree - q
  #print(f"位置误差:{position_error}")
  #根据步长调整关节角度,逐步接近目标速度
  #new_angle = q+joint_velocity*step_time
  #print(f"新的关节角度(弧度):{new_angle}")
  #将新的角度和速度转换为适合send_angle的数据格式
  for i in range(6):
    #将速度限制在0-100之间
    #取关节角速度的绝对值
    speed = int(np.clip(np.abs(joint_velocity[i]*10),1,100))
    #如果关节角速度是负值,调整目标角度的方向
    if joint_velocity[i]>=0:
      new_angle[i] = q[i]+joint_velocity[i]*step_time
    elif joint_velocity[i]<0:
      new_angle[i] = q[i]-joint_velocity[i]*step_time
    target_angle = float(target_degree[i]*180/np.pi)
    #mycobot.send_angle(i+1,target_angle,speed)
    #打印关节角速度以控制频率
    print(f"Step {step+1}/{steps}:关节{i+1}(deg): = {target_angle},关节角速度 ={speed}")
    mycobot.send_angle(i+1,target_angle,speed)
  time.sleep(step_time)


q_end = mycobot.get_radians()
print(f"最终的关节角度:{q_end}")
#打印关节角速度结果
print(f"最终关节角速度:{joint_velocity}")


