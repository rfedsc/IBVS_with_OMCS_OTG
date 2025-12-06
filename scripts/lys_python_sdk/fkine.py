import sympy as np
from calcST import calcST

#定义符号变量
theta1,theta2,theta3,theta4,theta5,theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
d1,d2,d3,d4,d5,d6 = sp.symbols('d1 d2 d3 d4 d5 d6')
a1,a2,a3,a4,a5,a6 = sp.symbols('a1 a2 a3 a4 a5 a6')
s1,s2,s3,s4,s5,s6 = sp.symbols('s1 s2 s3 s4 s5 s6')
c1,c2,c3,c4,c5,c6 = sp.symbols('c1 c2 c3 c4 c5 c6')
s23,c23,s234,c234 = sp.symbols('s23 c23 s234 c234')
p = sp.pi
r11,r12,r13,r14 = sp.symbols('r11 r12 r13 r14')
r21,r22,r23,r24 = sp.symbols('r21 r22 r23 r24')
r31,r32,r33,r34 = sp.symbols('r31 r32 r33 r34')


#计算矩阵 B
B = sp.Matrix([
    [c1,s1,0,0],
    [0,0,1,-d1],
    [s1,-c1,0,0],
    [0,0,0,1]
 ])*sp.Matrix([
    [r11,r12,r13,r14],
    [r21,r22,r23,r24],
    [r31,r32,r33,r34]，
    [0,0,0,1]
 ])

B = sp.simplify(B)

#计算正向运动学
T01 = sp.simplify(calcST(theta1,d1,0,p/2))
T12 = sp.simplify(calcST(theta2-p/2,0,a2,0))
T23 = sp.simplify(calcST(theta3,0,a3,0))
T34 = sp.simplify(calcST(theta4-p/2,d4,0,p/2))
T45 = sp.simplify(calcST(theta5+p/2,d5,0,-p/2))
T56 = sp.simplify(calcST(theta6,d6,0,0))

T16 = sp.simplify(T12*T23*T34*T45*T56)

T16 = T16.subs({
      sp.sin(theta1):s1,sp.sin(theta2):s2,sp.sin(theta3):s3,sp.sin(theta4):s4,
      sp.sin(theta5):s5,sp.sin(theta6):s6,
      sp.cos(theta1):c1,sp.cos(theta2):c2,sp.cos(theta3):c3,sp.cos(theta4):c4,
      sp.cos(theta5):c5,sp.cos(theta6):c6,
      sp.sin(theta2+theta3):s23,sp.cos(theta2+theta3):c23,
      sp.sin(theta2+theta3+theta4):s234,sp.cos(theta2+theta3+theta4):c234
      })


#显示结果
sp.pprint(T16)







