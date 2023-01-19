from sympy import *

# alpha1,alpha2,alpha3,alpha4,alpha5,alpha6,d1,d2,d3,d4,d5,d6,a1,a2,a3,a4,a5,a6 = symbols('alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6  ')
PI=3.14159265358979323846  
alpha1=(-90 * (PI/180))
alpha2=(0   * (PI/180))
alpha3=(-90 * (PI/180))
alpha4=(90  * (PI/180))
alpha5=(-90 * (PI/180))
alpha6=(0   * (PI/180))
d1= 0.0450
d2=0
d3=0
d4=0.640
d5=0
d6=0.095
a1=0.150
a2=0.570
a3=0.130
a4=0
a5=0
a6=0
tetha1,tetha2,tetha3,tetha4,tetha5,tetha6 = symbols('tetha1 tetha2 tetha3 tetha4 tetha5 tetha6')
init_printing(use_unicode=True)
T0_1 = Matrix([[cos(tetha1),(-1)*sin(tetha1)*cos(alpha1),sin(tetha1)*sin(alpha1),a1 * cos(tetha1)]
            ,[sin(tetha1),cos(tetha1)*cos(alpha1),(-1)*cos(tetha1)*sin(alpha1),a1 * sin(tetha1)]
            ,[0 , sin(alpha1),cos(alpha1),d1]
            ,[0,0,0,1]])
# print("first matrix:",T0_1)
T1_2 = Matrix([[cos(tetha2),(-1)*sin(tetha2)*cos(alpha2),sin(tetha2)*sin(alpha2),a2 * cos(tetha2)]
            ,[sin(tetha2),cos(tetha2)*cos(alpha2),(-1)*cos(tetha2)*sin(alpha2),a2 * sin(tetha2)]
            ,[0 , sin(alpha2),cos(alpha2),d2]
            ,[0,0,0,1]])
T2_3 = Matrix([[cos(tetha3),(-1)*sin(tetha3)*cos(alpha3),sin(tetha3)*sin(alpha3),a3 * cos(tetha3)]
            ,[sin(tetha3),cos(tetha3)*cos(alpha3),(-1)*cos(tetha3)*sin(alpha3),a3 * sin(tetha3)]
            ,[0 , sin(alpha3),cos(alpha3),d3]
            ,[0,0,0,1]])
T3_4 = Matrix([[cos(tetha4),(-1)*sin(tetha4)*cos(alpha4),sin(tetha4)*sin(alpha4),a4 * cos(tetha4)]
            ,[sin(tetha4),cos(tetha4)*cos(alpha4),(-1)*cos(tetha4)*sin(alpha4),a4 * sin(tetha4)]
            ,[0 , sin(alpha4),cos(alpha4),d4]
            ,[0,0,0,1]])
T4_5 = Matrix([[cos(tetha5),(-1)*sin(tetha5)*cos(alpha5),sin(tetha5)*sin(alpha5),a5 * cos(tetha5)]
            ,[sin(tetha5),cos(tetha5)*cos(alpha5),(-1)*cos(tetha5)*sin(alpha5),a5 * sin(tetha5)]
            ,[0 , sin(alpha5),cos(alpha5),d5]
            ,[0,0,0,1]])
T5_6 = Matrix([[cos(tetha6),(-1)*sin(tetha6)*cos(alpha6),sin(tetha6)*sin(alpha6),a6 * cos(tetha6)]
            ,[sin(tetha6),cos(tetha6)*cos(alpha6),(-1)*cos(tetha6)*sin(alpha6),a6 * sin(tetha6)]
            ,[0 , sin(alpha6),cos(alpha6),d6]
            ,[0,0,0,1]])
T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6

T0_6E = expand(T0_6)
# T0_6S = simplify(T0_6)
# T0_1S = simplify(T0_1)
# T0_1E = expand(T0_1S)
# print("type of matrix:",type(T0_6))
# print("size of matrix:",T0_6.shape)
# print("final matrix is :",T0_6E)
VP = T0_6E
VP = VP.row_del(3)
VP = VP.col_del(2)
VP = VP.col_del(1)
VP = VP.col_del(0)
VX = VP[0]
VY = VP[1]
VZ = VP[2]


# print("X pos is:",VP)

print("size of pos X is:",VX)
