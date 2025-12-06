import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
from fkine_6dof import fkine_6dof,transl,trotx,trotz
from jacob_cross_SDH import jacob_cross_sdh

#0.初始化
#轨迹起点
xyz_start = np.array([0,-120.02,533.96])
#轨迹终点
xyz_end = np.array([-30,-45,385])
#总时间
T = 10 

#1.轨迹规划
L = np.linalg.norm(xyz_end-xyz_start)
#每段时间
dt = T/7
#第一次加速度拐点
v1 = L/(16*dt)
#加加速度
J = 2*v1/(dt**2)
#最大加速度
amax = dt*J
#第二次加速度拐点
v2 = v1+dt*amax
#第三次加速度拐点
vmax = v2+v1

#轨迹时间点
t1 = 1*dt
t2 = 2*dt
t3 = 3*dt
t4 = 4*dt
t5 = 5*dt
t6 = 6*dt
t7 = 7*dt

#生成时间数组
t = np.arange(0,T+0.1,0.1)

#各阶段速度计算
vt1 = 0.5*J*t**2*(t>=0)*(t<t1)
vt2 = (v1+amax*(t-t1))*(t>=t1)*(t<t2)
vt3 = (vmax-0.5*J*(t3-t)**2)*(t>=t2)*(t<t3)
vt4 = vmax*(t>=t3)*(t<t4)
vt5 = (vmax-0.5*J*(t-t4)**2)*(t>=t4)*(t<t5)
vt6 = (v2-amax*(t-t5))*(t>=t5)*(t<t6)
vt7 = (0.5*J*(t7-t)**2)*(t>=t6)*(t<t7)

#总速度
vt = vt1+vt2+vt3+vt4+vt5+vt6+vt7
#计算位移
S = cumtrapz(vt,t,initial=0)
#各时刻xyz的位移
x_s = xyz_start[0]+(xyz_end[0]-xyz_start[0])/L*S
y_s = xyz_start[1]+(xyz_end[1]-xyz_start[1])/L*S
z_s = xyz_start[2]+(xyz_end[2]-xyz_start[2])/L*S

#各个时刻xyz轴的速度分量
v_x = (xyz_end[0]-xyz_start[0])/L*vt
v_y = (xyz_end[1]-xyz_start[1])/L*vt
v_z = (xyz_end[2]-xyz_start[2])/L*vt

#绘制各轴位移
plt.figure(figsize=(15,5))

plt.subplot(1,3,1)
plt.plot(t,x_s,'k',linewidth=1)
plt.xlabel('时间')
plt.ylabel('x轴位移')
plt.title('x轴')
plt.grid(True)

plt.subplot(1,3,2)
plt.plot(t,y_s,'k',linewidth=1)
plt.xlabel('时间')
plt.ylabel('y轴位移')
plt.title('y轴')
plt.grid(True)


plt.subplot(1,3,3)
plt.plot(t,x_s,'k',linewidth=1)
plt.xlabel('时间')
plt.ylabel('z轴位移')
plt.title('z轴')
plt.grid(True)

plt.show()

#2.轨迹跟踪
#使用指数趋近律的滑模控制
dth = np.zeros((6,len(t)))
th = np.zeros((6,len(t)+1))
x = np.zeros((6,len(t)+1))
x[:,0] = np.hstack((xyz_start,[0,0,0]))

lamda = 1
k = 0.1
ita = 0.0002
c = 5
alpha = 0

e = np.zeros((6,len(t)))
de = np.zeros((6,len(t)))

for i in range(len(t)):
  #期望位姿
  xd = np.hstack((x_s[i],y_s[i],z_s[i],[0,0,0]))
  #期望速度
  dxd = np.hstack((v_x[i],v_y[i],v_z[i],[0,0,0]))
  q = th[:,i]

  #x[:,i] = np.hstack((transl(fkine_6dof(q)),[0,0,0]))
  #求解当前角度下的雅可比矩阵
  Jac = jacob_cross_sdh(q) 
  e[:,i] = xd-x[:,i]
  s = c*e[:,i]
  v = dxd+(1/c)*ita*np.sign(s)
  de[:,i] = dxd-v
  #关节角度增量
  dth[:,i] = np.linalg.inv(Jac+lamda*np.eye(6))@v
  #下一时刻关节角度
  th[:,i+1] = th[:,i]+dth[:,i]*0.1
  #机械臂末端实际位姿
  x[:,i+1] = x[:,i]+v*0.1

#绘制误差曲线
plt.figure()
plt.plot(t,e[0:3,:].T,linewidth=2)
plt.xlabel('时间/s')
plt.ylabel('误差/mm')
plt.legend(['x轴的位置跟踪误差','y轴的位置跟踪误差','z轴的位置跟踪误差'])
plt.title('位置跟踪误差曲线')
plt.grid(True)
plt.show()

#绘制关节角度变化曲线
plt.figure(figsize=(15,10))
for i in range(6):
  plt.subplot(2,3,i+1)
  plt.plot(t,th[i,:-1],'k',linewidth=1.5)
  plt.xlabel('时间')
  plt.ylabel('角度值/rad')
  plt.title(f'第{i+1}关节角度变化曲线')
  plt.grid(True)

plt.show()


