#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Damped Last Square 
#para encontrar encontrar uma configuração q
#dada uma posição (x,y,z) e uma orientação (angulos de euler)
#no espaço para o Pioneer 7DOF


from math import cos, sin, sqrt, pi, atan2
import numpy as np
import random 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#Retorna a Matriz de transformacao Homogeneada usando como entrada os parametros de DH
def matriz_homogenea(d,a,alfa,theta):
    L1 = np.array([cos(theta), -sin(theta)*cos(alfa),\
                sin(theta)*sin(alfa),a*cos(theta)])
    L2 = np.array([sin(theta), cos(theta)*cos(alfa),\
                -cos(theta)*sin(alfa),a*sin(theta)])
    L3 = np.array([0,sin(alfa), cos(alfa), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

#matriz_antissimetrica
def S(a):
    #A = [0,-az,ay ; az,0,-ax ; -ay,ax,0]
    #Uso para calcular produto vetorial entre vetores u x v = S(u) * v
    A = np.zeros((3,3))
    A[0,1] = -a[2]
    A[0,2] = a[1]
    A[1,2] = -a[0]
    A[1,0] = - A[0,1]
    A[2,0] = - A[0,2]
    A[2,1] = - A[1,2]
    return A

def matriz_RPY(v):
    #v = (R,P,Y)
    A = np.zeros([3,3])
    A[0,0] = cos(v[0])*cos(v[1])
    A[0,1] = -sin(v[0])*cos(v[2]) + cos(v[0])*sin(v[1])*sin(v[2])
    A[0,2] = sin(v[0])*sin(v[2]) + cos(v[0])*sin(v[1])*cos(v[2])
    A[1,0] = sin(v[0])*cos(v[1])
    A[1,1] = cos(v[0])*cos(v[2]) + sin(v[0])*sin(v[1])*sin(v[2])
    A[1,2] = -cos(v[0])*sin(v[2]) + sin(v[0])*sin(v[1])*cos(v[2])
    A[2,0] = -sin(v[1])
    A[2,1] = cos(v[1])*sin(v[2])
    A[2,2] = cos(v[1])*cos(v[2]) 
    return A

#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2      
    return sqrt(d)

#Calcula os angulos de RPY a partir de uma matriz de Rotacao
def orientacao(A):
    #calcular os ângulos de orientação na conversão Z -> Y -> X
    R = atan2(A[1,0],A[0,0]) #Roll
    P = atan2(-A[2,0],sqrt((A[2,1]**2)+(A[2,2]**2))) #Pitch
    Y = atan2(A[2,1],A[2,2]) #Yaw
    result = np.array([[R,P,Y]]).T
    return result



#configurando o Rviz
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(100) # 100hz frequência de atualização
hello_str = JointState()
hello_str.header = Header()
hello_str.header.stamp = rospy.Time.now()
hello_str.name = ['One_joint', 'Two_joint', 'Three_joint', 'Four_joint'\
    ,'Five_joint','Six_joint','Seven_joint','L_F_joint','R_F_joint']
hello_str.position  = [0,0,0,0,0,0,0,0,0]    
hello_str.velocity = []
hello_str.effort = []

#variaveis do método
#passo maximo entre atualizacoes das juntas
qmax = 0.1 
#Constante alpha para melhorar  a aproximação
alfa = 0.1
#Constante de amortecimento
lbd = 0.05
I = np.eye(6)

#valor maximo que a junta pode assumir
qlim = [2.6179,1.5358,2.6179,1.6144,2.6179,1.8413,1.7889] 

#Objetivos
#posição desejada
pos_d = np.array([[0.2,0.2,0.3]]).T 
#Orientação desejada (RPY)
RPY_d = np.array([0,pi/2,0])
orient_d = matriz_RPY(RPY_d)

#vetores colunas do sistema de coordenadas global
k = np.array([[0,0,1,1]]).T
o = np.array([[0,0,0,1]]).T #origem

#Inicialização dos angulos de juntas
q = np.zeros([7,1])
for a in range(np.size(q)):
   q[a] = random.uniform(-qlim[a],qlim[a])

#Parâmetros Físicos do manipulador [m]
base = 0.05 #5 cm
L = 0.075 #distância da ultima junta a extremidade do efetuador

#parametros de DH constantes
d1 = 0.075 + base
d2 = 0
d3 = 0.15
d4 = 0 
d5 = 0.145
d6 = 0
d7 = 0
a1 = 0
a2 = 0
a3 = 0
a4 = 0
a5 = 0
a6 = 0.075
a7 = 0
alpha1 = pi/2
alpha2 = -pi/2
alpha3 = pi/2
alpha4 = -pi/2
alpha5 = pi/2
alpha6 = pi/2
alpha7 = pi/2

while not rospy.is_shutdown():
    for cont in range(1000):

        # parametros de DH variáveis
        theta1 = pi/2 + q[0]
        theta2 = q[1]
        theta3 = q[2]
        theta4 = q[3]
        theta5 = q[4]
        theta6 = pi/2 + q[5]
        theta7 = pi/2 + q[6]

        #Encontrando as matrizes homogêneas atuais
        A1 = matriz_homogenea(d1,a1,alpha1,theta1)
        A2 = matriz_homogenea(d2,a2,alpha2,theta2)
        A3 = matriz_homogenea(d3,a3,alpha3,theta3)
        A4 = matriz_homogenea(d4,a4,alpha4,theta4)
        A5 = matriz_homogenea(d5,a5,alpha5,theta5)
        A6 = matriz_homogenea(d6,a6,alpha6,theta6)
        A7 = matriz_homogenea(d7,a7,alpha7,theta7)

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
        p_0 = T7@p_7

        #atualiaza os angulos do manipulador no Rviz
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0],0,0]
        pub.publish(hello_str)
        rate.sleep()        

        #Condição de parada   
        erro =  distancia(p_0,pos_d,3)   
        if(erro < 0.001):
            print('Solucao q: \n',q,'\nNumero de iteracoes:',cont)
            break      

        #os vetores z serao transformados em vetores  no R^3
        z0_0 = k[0:3]
        z1_0 = (T1@k)[0:3]
        z2_0 = (T2@k)[0:3]
        z3_0 = (T3@k)[0:3]
        z4_0 = (T4@k)[0:3]
        z5_0 = (T5@k)[0:3]
        z6_0 = (T6@k)[0:3]
        #z7_0 = (T7@k)[0:3] nao eh usado

        #cálculo do Jacobiano geometrico
        J = np.zeros([6,7])
        #produto vetorial de Z0_0 por (o7_0 - o) 
        J[0:3,0] = S(z0_0)@(o7_0[0:3] - o[0:3])[:,0]
        J[3:6,0] = z0_0[:,0]
        J[0:3,1] = S(z1_0)@(o7_0[0:3] - o1_0[0:3])[:,0]
        J[3:6,1] = z1_0[:,0]
        J[0:3,2] = S(z2_0)@(o7_0[0:3] - o2_0[0:3])[:,0]
        J[3:6,2] = z2_0[:,0]
        J[0:3,3] = S(z3_0)@(o7_0[0:3] - o3_0[0:3])[:,0]
        J[3:6,3] = z3_0[:,0]
        J[0:3,4] = S(z4_0)@(o7_0[0:3] - o4_0[0:3])[:,0]
        J[3:6,4] = z4_0[:,0]
        J[0:3,5] = S(z5_0)@(o7_0[0:3] - o5_0[0:3])[:,0]
        J[3:6,5] = z5_0[:,0]
        J[0:3,6] = S(z6_0)@(o7_0[0:3] - o6_0[0:3])[:,0]
        J[3:6,6] = z6_0[:,0]
       
        #Calculo da erro
        f = np.zeros([6,1])
        f[0:3] = pos_d - p_0[0:3]
        #a parte angular peguei do artigo A closed-loop inverse kinematic scheme
        #online joint based robot control
        f[3:6] = 0.5*(S(T7[0:3,0:1])@orient_d[:,0:1]+ S(T7[0:3,1:2])@orient_d[:,1:2]  + \
            S(T7[0:3,2:3])@orient_d[:,2:3])
        
        #Equação do DLS
        dq = alfa*((J.T@np.linalg.inv(J@J.T + lbd*I))@f)

        #limitando o delta q
        for i2 in range(np.size(dq)):
            if(dq[i2] > qmax):
                dq[i2] = qmax
            elif(dq[i2] < -qmax):
                dq[i2] = -qmax 

        #Atualizando a cofiguração
        q = q + dq

        #Limitando os valores das juntas
        for i2 in range(np.size(q)):
            if(q[i2] > qlim[i2]):
                q[i2] = qlim[i2]
            elif(q[i2] < -qlim[i2]):
                q[i2] = -qlim[i2]
        

    break   
print(erro) 
print(q)
print(np.round(T7[0:3,0:3],2))
# %%
