import time
import numpy as np
from mycobot_init import MyCobot_Init


#计算圆心
def calculate_circle_center(p1,p2,p3):
  temp = p2[0]**2+p2[1]**2
  bc = (p1[0]**2+p1[1]**2-temp)/2
  cd = (temp-p3[0]**2-p3[1]**2)/2
  det = (p1[0]-p2[0])*(p2[1]-p3[1])-(p2[0]-p3[0])*(p1[1]-p2[1])
  if abs(det)<1.0e-10:
    raise ValueError("Points are collinear")
  cx = (bc*(p2[1]-p3[1])-cd*(p1[1]-p2[1]))/det
  cy = ((p1[0]-p2[0])*cd-(p2[0]-p3[0])*bc)/det
  return np.array([cx,cy])

#生成圆弧轨迹
def generate_arc_trajectory(start,middle,end,steps):
  #计算圆心
  center = calculate_circle_center(start[:2],middle[:2],end[:2])
  #计算半径
  radius = np.linalg.norm(start[:2]-center)
  #计算起始角
  start_angle = np.arctan2(start[1]-center[1],start[0]-center[0])
  #计算结束角度
  end_angle = np.arctan2(end[1]-center[1],end[0]-center[0])
  #生成圆弧轨迹
  trajectory = []
  for i in range(steps):
    angle = start_angle + i*(end_angle - start_angle)/(steps-1)
    x = center[0] + radius*np.cos(angle)
    y = center[1] + radius*np.sin(angle)
    z = start[2] + i*(end[2]-start[2])/(steps-1)
    trajectory.append([x,y,z,start[3],start[4],start[5]])
  return trajectory

#执行轨迹
def execute_trajectory(mycobot,trajectory,speed):
  for point in trajectory:
   # for i in range(20):
     mycobot.send_coords(point,speed,mode=1) 
     time.sleep(0.1)

def main():
  mycobot = MyCobot_Init()
  #获取当前坐标
  current_coords = mycobot.get_coords()
  print(f"Current Coords:{current_coords}")
  start_coords = current_coords
  middle_coords = [10,20,20,0,0,0]
  end_coords = [8080,0,10,0,0,0]
  steps = 50;
  speed = 50;

  #生成圆弧轨迹
  trajectory = generate_arc_trajectory(start_coords,middle_coords,end_coords,steps)
  
  #执行圆弧轨迹
  execute_trajectory(mycobot,trajectory,speed)

if __name__ == "__main__":
  main()









