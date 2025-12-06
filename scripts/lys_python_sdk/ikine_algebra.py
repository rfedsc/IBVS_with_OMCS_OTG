import numpy as np
import math
from sympy import symbols,pi,acos,asin,atan2,cos,sin,Matrix
from calcST import calcST
from mycobot_init import joints_limit

#np.set_printoptions(suppress=True)
def InverseKinematics(T,para):
  #初始化一个空的列表,用于存储所有可能的关节角度解
  theta = []
  theta_ = []
  #从para参数中提取机械臂的DH参数
  d1,a2,a3,d4,d5,d6 = para

  print(f"para is {para}")

  #e = r14-d6*r13
  e = T[0,3] - d6*T[0,2]
  #print(f"e is {e}")
  #f = r24-d6*r23
  f = T[1,3] - d6*T[1,2]
  #print(f"f is {f}")
  #方程:g*c1^2+hc1+k = 0
  #g,h,k是中间变量,用于计算三次方程的系数
  #g = e^2+f^2
  g = e**2 + f**2
  #h = 2*d4*f
  h = 2*d4*f
  #k = d4^2-e^2
  k = d4**2-e**2
  #用于一个二次方程是否有实数根的判别式
  delta = h**2 - 4*g*k
  
  for ic1 in range(2):
    if ic1 == 0:
      #求根公式,用于计算c1的第一个解
      c1 = (-h+np.sqrt(delta))/(2*g)
    else:
      #求根公式,用于计算c1的第二个解
      c1 = (-h-np.sqrt(delta))/(2*g)
    
    for i1 in range(2):
      if i1 == 0:
        #用于计算theta1的第一个数值
        theta1 = acos(c1)
        #print(f'tehta1-1:{theta1}')
      else:
        #用于计算theta1的第二个数值
        theta1 = -acos(c1)
        #print(f'theta1-2:{theta2}')
      #B = T16 = T10*T06
      B = Matrix([
          [cos(theta1),sin(theta1),0,0],
          [0,0,1,-d1],
          [sin(theta1),-cos(theta1),0,0],
          [0,0,0,1]
      ])*T
      
      for i5 in range(2):
        if i5 == 0:
          # -s5 = b33 =>-b33 = s5
          theta5 = asin(-B[2,2])
         # print(f'tehta5-1:{theta5}')
        else:
          #如果-B[2,2]>=0,theta5取正解的补角
          if -B[2,2] >= 0:
            theta5 = math.pi - asin(-B[2,2])
            #print(f'tehta5-2-1:{theta5}') 
          #如果-B[2,2]<0,theta5取负解的补角
          else:
            theta5 = -math.pi - asin(-B[2,2])
            #print(f'tehta5-2-2:{theta5}')
        #如果cos(theta5)>=0,使用atan2(-B[2,2],B[2,0])来计算theta6
        #theta6 = arctan2(-b32,b31)/theta6 = arctan2(b32,-b31)
        if cos(theta5) >= 0:
          theta6 = math.atan2(-B[2,1],B[2,0])
          #print(f'tehta6-1-1:{theta6}')
        #如果cos(theta5)<0,使用atan2(B[2,1],-B[2,0])来计算theta6
        else:
          theta6 = math.atan2(B[2,1],-B[2,0])
          #print(f'tehta6-1-2:{theta6}')
        
        #多个可能的theta234解
        #theta234 = arctan2()
        for i234 in range(6):
          if i234 == 0:
            #对应一个偏移为-2pi的解
            theta234 = acos(B[0,2]/cos(theta5)) - 2*math.pi
           # print(f'tehta234-1:{theta234}')
          elif i234 == 1:
            #直接的解
            theta234 = acos(B[0,2]/cos(theta5))
           # print(f'tehta234-2:{theta234}')
          elif i234 == 2:
            #对应一个偏移为+2pi的解
            theta234 = acos(B[0,2]/cos(theta5)) + 2*math.pi
           # print(f'tehta234-3:{theta234}')
          elif i234 == 3:
            #考虑acos的负解并+2pi
            theta234 = -acos(B[0,2]/cos(theta5)) + 2*math.pi
           # print(f'tehta234-4:{theta234}')
          elif i234 == 4:
            #负解的直接形式
            theta234 = -acos(B[0,2]/cos(theta5))
           # print(f'tehta234-5:{theta234}')
          else:
            #负解-2pi
            theta234 = -acos(B[0,2]/cos(theta5)) - 2*math.pi
           # print(f'tehta234-6:{theta234}')
         
          #m = b14 + d5*s234-d6*c5*c234
          m = B[0,3] + d5*sin(theta234) - d6*cos(theta5)*cos(theta234)
          #n = b24 -d5*c234-d6*c5*s234
          n = B[1,3] - d5*cos(theta234) - d6*cos(theta5)*sin(theta234)
          #p = m^2+n^2+a2^2-a3^2
          p = m**2 + n**2 +a2**2 -a3**2
          #q = 2*n*a2
          q = 2*n*a2
          #r = 2*m*a2
          r = 2*m*a2
          #方程:(q^2+r^2)*c2^2+2pq*c2+p^2-r^2 = 0
          #即:s*c2^2+t*c2+u = 0
          #s = q^2+r^2
          s = q**2+r**2
          #t = 2*p*q
          t = 2*p*q
          #u = p^2-r^2
          u = p**2 - r**2
          #delta2用于判断二次方程是否有实数解
          delta2 = t**2 - 4*s*u
          #如果delta2<0,说明方程没有实数解,此时跳过当前循环
          #继续下一次迭代      
          if delta2 < 0:
            continue

          #求解第二个关节角度theta2以及中间角度c23
          for ic2 in range(2):
            if ic2 == 0:
              #c2的第一个解
              c2 = (-t + math.sqrt(delta2))/(2*s)
            else:
              #c2的第二个解
              c2 = (-t - math.sqrt(delta2))/(2*s)
         
            #计算theta2 
            for i2 in range(2):
              if i2 == 0:
                theta2 = acos(c2)
               # print(f'tehta2-1:{theta2}')
              else:
                theta2 = -acos(c2)
               # print(f'tehta2-2:{theta2}')

              #计算中间角度c23
              c23 = (n+a2*cos(theta2))/(-a3)

              for i23 in range(4):
                #标准的反余弦值,结果是[0,pi]之间的角度
                if i23 == 0:
                  theta23 = acos(c23)
                 # print(f'tehta23-1:{theta23}')
                #[pi,2pi]之间的角度
                elif i23 == 1:
                  theta23 = 2*math.pi - acos(c23)
                 # print(f'tehta23-2:{theta23}')
                #考虑的是负角度的情况
                elif i23 == 2:
                  theta23 = -acos(c23)
                 # print(f'tehta23-3:{theta23}')
                #负周期内的对称角,[-2pi,-pi]之间的角度
                else:
                  theta23 = -2*math.pi+acos(c23)
                 # print(f'tehta23-4:{theta23}')
           
                #如果theta234-theta23<=pi
                #则他们的差值可以直接用作theta4 
                if abs(theta234-theta23) <= math.pi:
                  theta4 = theta234-theta23
                 # print(f'tehta4:{theta4}')
                else:
                  #如果差值不在这个范围内,则认为改组不合理
                  #跳过当前的解
                  continue
                #检查theta23-theta2<=pi
                #则他们的差值直接作为theta3 
                if abs(theta23-theta2) <= math.pi:
                  theta3 = theta23 - theta2
                 # print(f'tehta3:{theta3}')
                else:
                #如果差值不在这个范围内,则认为改组不合理
                #跳过当前的解
                  continue

                T1 = calcST(theta1,d1,0,pi/2)
                T2 = calcST(theta2-pi/2,0,a2,0)
                T3 = calcST(theta3,0,a3,0)
                T4 = calcST(theta4-pi/2,d4,0,pi/2)
                T5 = calcST(theta5+pi/2,d5,0,-pi/2)
                T6 = calcST(theta6,d6,0,0)
                T_target = T1*T2*T3*T4*T5*T6
               # print('**************************')
               # print(f'T = {T}')
               # print(f'T_target = {T_target}')
                error=np.linalg.norm(np.array(T)-np.array(T_target),ord='fro')
                print(f'{error}')
               # print('**************************')
                #检查解是否已经存在
                solution = [theta1,theta2,theta3,theta4,theta5,theta6]
                limits = joints_limit(solution)
                if solution not in theta_ and limits :
                  theta_.append(solution) 
                  print(f'theta1-6:{solution}')

 # return theta
                #print(f'tehta1-6:{theta1},{theta2},{theta3},{theta4},{theta5},{theta6}')
                #用于验证一组关节角度(theta1到theta6)是否能够使机械臂末端
                #执行器达到给定的目标位置T,通过比较计算出来的末端执行器件
                #位置T_end和目标位置T,可以确定当前角度组合的正确性           
                if limits and np.linalg.norm(np.array(T)-np.array(T_target),ord='fro')  < 1.72:
                  theta.append([theta1,theta2,theta3,theta4,theta5,theta6])
                  #返回theta
  return theta

