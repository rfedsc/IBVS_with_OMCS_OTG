from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT,PI_BAUD
import numpy as np
import math
import time

def MyCobot_Init():
  mycobot = MyCobot(PI_PORT,PI_BAUD)
  return mycobot

def mycobot_release():
    mycobot = MyCobot_Init()
    mycobot.release_all_servos()
    print("放松完成")

def joints_limit(q):
  J1=[-168*math.pi/180,168*math.pi/180]
  J2=[-135*math.pi/180,135*math.pi/180]
  J3=[-150*math.pi/180,150*math.pi/180]
  J4=[-145*math.pi/180,145*math.pi/180]
  J5=[-165*math.pi/180,165*math.pi/180]
  J6=[-180*math.pi/180,180*math.pi/180]
  if (J1[0]<=q[0]<=J1[1] and 
      J2[0]<=q[1]<=J2[1] and 
      J3[0]<=q[2]<=J3[1] and
      J4[0]<=q[3]<=J4[1] and 
      J5[0]<=q[4]<=J5[1] and 
      J6[0]<=q[5]<=J6[1]):
      ret = True
  else:
      ret = False
  
  return ret








def test():
  mycobot = MyCobot_Init()
  mycobot_release()
  '''
  while True:
    flag = mycobot.is_in_position([10,-10,10,-10,10,-10],0)
    print(flag)
    if flag == 0:
      mycobot.send_angles([10,-10,10,-10,10,-10],50)
      time.sleep(0.1)
      print("未达到目标位置")
    elif flag == 1:
      print("达到目标位置")
  '''
  mycobot.send_coords([0,0,0,0,0,0],50,1)
  time.sleep(3)
  #for i in range(20):
  #  mycobot.send_coords([50,50,50,50,50,50],50,1)
  #  time.sleep(0.1)
  theta = [1.00000000000000, -1.00000000000000, -0.999999999999992, -1.28318530717959, 1.00000000000000, 1.0000000000000002]
  theta1 = [1.0000, 1.9238, -1.0000, 2.0762, 1.0000, 1.0000] 
  theta2 = [-1.4999, -1.7113, -1.1341, 2.0528, -1.6781, -1.2324]
  theta3 = [1.49992248395476, -0.864094636455354, -1.63989023256690, -1.49440307989419, 1.46973428899964, 0.265115758477796]
  theta_degree = [np.degrees(-1.4999),-130,np.degrees(1.1341),np.degrees(0.8298),np.degrees(-1.6781),np.degrees(-1.2324)]
  theta4 = [-1.00000000000000, 0.737153267179906, 1.53600964226011, 0.728809382246938, -0.990963664839014, 1.2364061845509535]
  theta5 = [-1.00000000000000, 0.553820852118296, 1.79271943140349, 0.934672731970848, -0.990963664839014, 1.2364061845509535]
  angles = [0,-9.5,86,-165,0,-90]
  angles2 = [-145.8492,135.0000   -0.0000 -145.0000 -145.8492   -0.0000]
  #mycobot.send_angles(angles,50)
  #print(theta_degree)
  #mycobot.send_radians(theta3,50)
 # mycobot.send_angles(theta_degree,50)
 # mycobot.send_radians(theta5,50)
 #mycobot.send_radians([0,-1.0856,0.3170,3.1415,-0.7686,0],50)
 # mycobot.send_radians([0.086,0.104,-0.274,0.157,0.089,-0.005],50)
  time.sleep(3)
  T = mycobot.get_coords()
  print(T)

  


if __name__ == "__main__":
  test()
# mycobot_release()
