# %%
from math import cos, sin, sqrt, pi, atan2
import numpy as np
import math
import random 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
#transforma uma matriz A m x n em uma matriz  3 x 7
def M_3x7(A):
    L1 = np.array([A[0,0],A[0,1],A[0,2],A[0,3],A[0,4],A[0,5],A[0,6]])
    L2 = np.array([A[1,0],A[1,1],A[1,2],A[1,3],A[1,4],A[1,5],A[1,6]])
    L3 = np.array([A[2,0],A[2,1],A[2,2],A[2,3],A[2,4],A[2,5],A[2,6]])
    H = np.array([L1,L2,L3])
    return H
                    
#transforma um vetor v n x 1 em um vetor 3 x 1
def v_3d(v):
    return np.array([v[0],v[1],v[2]])

#Retorna a Matriz de transformacao Homogeneadados  usando como entrada os parametros de DH
def matriz_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2      
    return sqrt(d)

#Calcula os angulos de Euler a partir de uma matriz de Rotacao
def orientacao(A):
    #Z-Y-Z Euler Angles
    if(A[0,2] != 0 or A[1,2] !=0):
        theta = atan2(sqrt(1 - A[2,2]**2),A[2,2])
        phi = atan2(A[1,2],A[0,2])
        psi = atan2(A[2,1],-A[2,0])
    else:
        if(A[2,2] == 1):
            theta = 0
            phi = 0
            psi = atan2(A[0,0],A[1,0])
        else:
            theta = pi
            phi = 0
            psi = - atan2(-A[0,0],-A[0,1]) 
    result = np.array([theta,phi,psi])
    return result

def matriz_B(v): #Matriz B(alpha) da equação 4.108 do livro Spong-RobotmodelingandControl
    L1 = np.array([round(cos(v[2])*sin(v[0]),4),round(-sin(v[2]),4),0])
    L2 = np.array([round(sin(v[2])*sin(v[0]),4),round(cos(v[2]),4),0])
    L3 = np.array([round(cos(v[0]),4),0,1])
    A = np.array([L1,L2,L3])
    return A

def Ja(J,B):#Usando como entrada a matriz B(alpha) e a Jacobiana analitica eh calculada a matriz geometrica
    L1 = np.array([1,0,0,0,0,0])
    L2 = np.array([0,1,0,0,0,0])
    L3 = np.array([0,0,1,0,0,0])
    C = np.linalg.inv(B)
    L4 = np.array([0,0,0,C[0,0],C[0,1],C[0,2]])
    L5 = np.array([0,0,0,C[1,0],C[1,1],C[1,2]])
    L6 = np.array([0,0,0,C[2,0],C[2,1],C[2,2]])
    A = np.array([L1,L2,L3,L4,L5,L6])
    return A@J

#configurando o plot

pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(10) # 10hz
hello_str = JointState()
hello_str.header = Header()
hello_str.header.stamp = rospy.Time.now()
hello_str.name = ['One_joint', 'Two_joint', 'Three_joint', 'Four_joint'\
    ,'Five_joint','Six_joint','Seven_joint']
hello_str.position  = [0,0,0,0,0,0,0]    
hello_str.velocity = []
hello_str.effort = []

#variaveis
cont = 0
qmax = 0.1
c = 0.5 #tamanho do passo
destino = np.array([[0.2,0.5,0.7]]).T

#vetores colunas do sistema de coordenadas global
i = np.array([[1,0,0,1]]).T
j = np.array([[0,1,0,1]]).T
k = np.array([[0,0,1,1]]).T
o = np.array([[0,0,0,1]]).T #origem

#angulos de juntas iniciais
q1 = 2*pi*random.random()
q2 = 2*pi*random.random()
q3 = 2*pi*random.random()
q4 = 2*pi*random.random()
q5 = 2*pi*random.random()
q6 = 2*pi*random.random()
q7 = 2*pi*random.random()
q = np.array([[q1,q2,q3,q4,q5,q6,q7]]).T

#Comprimento dos elos do manipulador
b1 = 0.2 #20 cm
b2 = 0.1
b3 = 0.2 
b4 = 0.1
b5 = 0.2
b6 = 0.1
b7 = 0.2
L = 0.2 #comprimento do elo que liga a junta 7 ao efetuador
### parametros de DH constantes
d1 = b1 + b2
d2 = 0
d3 = b3 + b4
d4 = 0 
d5 = b5 + b6
d6 = 0
d7 = 0
a1 = 0
a2 = 0
a3 = 0
a4 = 0
a5 = 0
a6 = b7
a7 = 0
alpha1 = pi/2
alpha2 = -pi/2
alpha3 = pi/2
alpha4 = -pi/2
alpha5 = pi/2
alpha6 = pi/2
alpha7 = pi/2

while not rospy.is_shutdown():
    for v in range(100000):
        
        # parametros de DH variáveis
        theta1 = pi/2 + q[0]
        theta2 = q[1]
        theta3 = q[2]
        theta4 = q[3]
        theta5 = q[4]
        theta6 = pi/2 + q[5]
        theta7 = pi/2 + q[6]

        #Matrizes homogêneas
        A1 = matriz_homogenea(d1,a1,alpha1,theta1)
        A2 = matriz_homogenea(d2,a2,alpha2,theta2)
        A3 = matriz_homogenea(d3,a3,alpha3,theta3)
        A4 = matriz_homogenea(d4,a4,alpha4,theta4)
        A5 = matriz_homogenea(d5,a5,alpha5,theta5)
        A6 = matriz_homogenea(d6,a6,alpha6,theta6)
        A7 = matriz_homogenea(d7,a7,alpha7,theta7)
        A = A1@(A2@(A3@(A4@(A5@(A6@A7)))))

        #Definindo os pontos de interesse em seus sistemas locais
        o1_1 = o #origem do SC 1
        o2_2 = o #origem do SC 2
        o3_3 = o #origem do SC 3
        o4_4 = o #origem do SC 4
        o5_5 = o #origem do SC 5
        o6_6 = o #origem do SC 6
        o7_7 = o #origem do SC 7
        p_7 = np.array([[0,0,L,1]]).T
        #Calculando os pontos de interesse no sistema Global
        T1 = A1
        T2 = T1@A2
        T3 = T2@A3
        T4 = T3@A4
        T5 = T4@A5
        T6 = T5@A6
        T7 = T6@A7
        o1_0 = T1@o1_1
        o2_0 = T2@o2_2
        o3_0 = T3@o3_3
        o4_0 = T4@o4_4
        o5_0 = T5@o5_5
        o6_0 = T6@o6_6
        o7_0 = T7@o7_7
        p_0 = A@p_7
        #os vetores z serao transformados em vetores linhas no R^3, para poder ser usado a funcao np.cross
        z0_0 = v_3d(k).T
        z1_0 = v_3d(T1@k).T
        z2_0 = v_3d(T2@k).T
        z3_0 = v_3d(T3@k).T
        z4_0 = v_3d(T4@k).T
        z5_0 = v_3d(T5@k).T
        z6_0 = v_3d(T6@k).T
        #z7_0 = v_3d(T7@k).T nao eh usado

        aux = (v_3d(o7_0) - v_3d(o)).T
        aux = np.cross(z0_0,aux)
        J = np.array([np.concatenate((aux,z0_0),axis = None)])

        aux = (v_3d(o7_0) - v_3d(o1_0)).T
        aux = np.cross(z1_0,aux)
        J2 = np.array([np.concatenate((aux,z1_0),axis = None)])
        J = np.concatenate((J,J2),axis = 0)

        aux = (v_3d(o7_0) - v_3d(o2_0)).T
        aux = np.cross(z2_0,aux)
        J3 = np.array([np.concatenate((aux,z2_0),axis = None)])
        J = np.concatenate((J,J3),axis = 0)

        aux = (v_3d(o7_0) - v_3d(o3_0)).T
        aux = np.cross(z3_0,aux)
        J4 = np.array([np.concatenate((aux,z3_0),axis = None)])
        J = np.concatenate((J,J4),axis = 0)

        aux = (v_3d(o7_0) - v_3d(o4_0)).T
        aux = np.cross(z4_0,aux)
        J5 = np.array([np.concatenate((aux,z4_0),axis = None)])
        J = np.concatenate((J,J5),axis = 0)

        aux = (v_3d(o7_0) - v_3d(o5_0)).T
        aux = np.cross(z5_0,aux)
        J6 = np.array([np.concatenate((aux,z5_0),axis = None)])
        J = np.concatenate((J,J6),axis = 0)

        aux = (v_3d(o7_0) - v_3d(o6_0)).T
        aux = np.cross(z6_0,aux)
        J7 = np.array([np.concatenate((aux,z6_0),axis = None)])
        J = np.concatenate((J,J7),axis = 0)

        J = J.T
        f = (v_3d(p_0) - destino)
        J = M_3x7(J) #foi descartado a parte da velocidade angular, porque ela nao esta sendo usada
        #e caso estivesse sendo, teria que ser convertida em taxa de angulo.
        e = np.array([f[0,0],f[1,0],f[2,0]])
        c = e.dot(J@J.T@e)/((J@J.T@e).dot(J@J.T@e))#peguei essa equacao do
        #artigo Inverse Kinematics Techniques in Computer Graphics: A Survey
        dq = - c*(J.T@f)
        for i2 in range(np.size(dq)):
            if(dq[i2] > qmax):
                dq[i2] = qmax
            elif(dq[i2] < -qmax):
                dq[i2] = -qmax 
        q = q + dq
        erro = distancia(p_0,destino,3)
        #print('erro: ',erro,'\n','iteracao: ',v,'\n') 
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0]]
        pub.publish(hello_str)
        rate.sleep()
        if(erro < 0.01):
            destino = np.array([[0.5*random.random(),0.5*random.random(),0.5*random.random()]]).T
            #print('Solucao q: \n',q,'\nNumero de iteracoes:',v)
            #break
    break    




    

# %%
