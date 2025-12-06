import numpy as np
from dh import d,a,alp,offset
np.set_printoptions(suppress=True)

def fkine_6dof_mdh(q):

    thd = q+offset
    #计算各个关节的变换矩阵
    A_matrices = []
    for i in range(6):
        A = np.array([
                    [np.cos(thd[i])               ,-np.sin(thd[i])              ,0              ,a[i]                ],
                    [np.sin(thd[i])*np.cos(alp[i]),np.cos(thd[i])*np.cos(alp[i]),-np.sin(alp[i]),-d[i]*np.sin(alp[i])],
                    [np.sin(thd[i])*np.sin(alp[i]),np.cos(thd[i])*np.sin(alp[i]),np.cos(alp[i]) ,d[i]*np.cos(alp[i]) ],
                    [0                            ,0                            ,0              ,1                   ]
                    ])
        A_matrices.append(A)
    #计算最终变换矩阵
    T = np.eye(4)

    for A in A_matrices:
        T = T@A

    return T

def test():
    #使用零角度作为示例
    q = np.zeros(6)
    T = fkine_6dof_mdh(q)
    print("最终变换矩阵:")
    print(T)

if __name__ == "__main__":
    test()



