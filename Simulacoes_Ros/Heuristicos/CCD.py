#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Cyclic Coordinate Descent para encontrar uma configuação q 
#dada uma posição (x,y,z) no espaço para o Pioneer 7DOF

#Import do modulo funcoes.py
import sys
sys.path.append('/home/david/Pibic2021/Inverse-Kinematic-of-Serial-Manipulator-Robots/Simulacoes_Ros')
from funcoes import matriz_homogenea, random_pose, distancia, Cinematica_Direta

#Import das bibliotecas python
from math import cos, sin, sqrt, pi, acos
import numpy as np
import random 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#realiza a operação de multiplicação de quaternios
def multiplicacao_quaternios(q,q2):  
    resultado = np.zeros([4,1])
    resultado[0,0] = q[0,0]*q2[0,0] -q[1,0]*q2[1,0] -q[2,0]*q2[2,0] -q[3,0]*q2[3,0] 
    resultado[1,0] = q[0,0]*q2[1,0] +q[1,0]*q2[0,0] +q[2,0]*q2[3,0] -q[3,0]*q2[2,0] 
    resultado[2,0] = q[0,0]*q2[2,0] -q[1,0]*q2[3,0] +q[2,0]*q2[0,0] +q[3,0]*q2[1,0] 
    resultado[3,0] = q[0,0]*q2[3,0] +q[1,0]*q2[2,0] -q[2,0]*q2[1,0] +q[3,0]*q2[0,0] 
    return resultado

#gira p em torno de v em th rad
def rotationar_vetor(p,v,th):
    a = cos(th/2)
    b = v[0,0]*sin(th/2)
    c = v[1,0]*sin(th/2)
    d = v[2,0]*sin(th/2)
    p_aumentado = np.zeros([4,1])
    p_aumentado[1:4,0] = p[:,0]
    h = np.array([[a,b,c,d]]).T
    hx = np.array([[a,-b,-c,-d]]).T
    p_r = multiplicacao_quaternios(h,p_aumentado)
    q_r = multiplicacao_quaternios(p_r,hx)
    return q_r[1:4]

#Calcula o acos de um angulo arrendondado em 10 casas decimais
def acosr(x):
    return acos(np.round(x,10))

#Calcula a norma de um vetor 
def norm(v):
    return sqrt(v[[0]]**2 + v[[1]]**2 + v[[2]]**2)

#criar um vetor coluna a partir de uma lista
def vetor(v):  
    return np.array([[v[0],v[1],v[2]]]).T

#Projeta um ponto em um plano
def projecao_ponto_plano(normal,p0,p):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    #constante d da equação do 
    d = -normal.T@p0 #produto escala 
    #distancia do ponto ao plano
    alpha = (-d - normal.T@p)/(normal[0,0]**2 +  normal[1,0]**2 + normal[2,0]**2)
    ponto_projetado = p + alpha*normal
    return ponto_projetado

#configurando o Rviz
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(100) # 100hz 
hello_str = JointState()
hello_str.header = Header()
hello_str.header.stamp = rospy.Time.now()
hello_str.name = ['One_joint', 'Two_joint', 'Three_joint', 'Four_joint'\
    ,'Five_joint','Six_joint','Seven_joint','L_F_joint','R_F_joint']
hello_str.position  = [0,0,0,0,0,0,0,0,0]    
hello_str.velocity = []
hello_str.effort = []

[posicaod,orientd] = random_pose()
print(np.round(posicaod,4))
q = np.array([[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]).T
n = 7 #número de juntas

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



for k in range(25):
    # parametros de DH variáveis
    theta1 = pi/2 + q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    theta5 = q[4]
    theta6 = pi/2 + q[5]
    theta7 = pi/2 + q[6]

    #Calculando as matrizes homogêneas
    A1 = matriz_homogenea(d1,a1,alpha1,theta1)
    A2 = matriz_homogenea(d2,a2,alpha2,theta2)
    A3 = matriz_homogenea(d3,a3,alpha3,theta3)
    A4 = matriz_homogenea(d4,a4,alpha4,theta4)
    A5 = matriz_homogenea(d5,a5,alpha5,theta5)
    A6 = matriz_homogenea(d6,a6,alpha6,theta6)
    A7 = matriz_homogenea(d7,a7,alpha7,theta7)

    #Calculando os pontos de interesse e vetores no sistema Global
    T1 = A1
    T2 = T1@A2
    T3 = T2@A3
    T4 = T3@A4
    T5 = T4@A5
    T6 = T5@A6
    T7 = T6@A7

    #vetores de atuação das juntas
    Vy = np.array([T1[0:3,1],T2[0:3,1],T3[0:3,1],T4[0:3,1],T5[0:3,1],T6[0:3,1],T7[0:3,1]]).T

    pontos = Cinematica_Direta(q) 
    erro = distancia(pontos[:,7],posicaod,3) 

    for i in range(n-1,-1,-1):
        pontos = Cinematica_Direta(q) 
        proj = projecao_ponto_plano(vetor(Vy[:,i]),pontos[:,i],posicaod[:])
        va = proj - vetor(pontos[:,i])
        va = va/norm(va)
        proj = projecao_ponto_plano(vetor(Vy[:,i]),pontos[:,i],vetor(pontos[:,7]))
        vb = proj - vetor(pontos[:,i])
        vb = vb/norm(vb)
        th = acosr(va.T@vb)
        j = i + 1
        if(j == 4 or j == 2):#Se for a junta 2 e 4
            v = rotationar_vetor(va,vetor(Vy[:,i]),pi/2)
        else:
            v = rotationar_vetor(va,vetor(Vy[:,i]),-pi/2) 

        if(vb.T@v < 0): th = -th
        q[i,0] = q[i,0] + th

        pontos = Cinematica_Direta(q)
        erro_anterior = erro
        erro = distancia(pontos[:,7],posicaod,3) 
        if(erro_anterior < erro):
            print('Deu ruim',j)
            break

    pontos = Cinematica_Direta(q)
    erro = distancia(pontos[:,7],posicaod,3) 

    if(erro < 10**-5):
        print(erro,'\n','fim da interacao:',k,'\n')
        break

#atualiaza os angulos do manipulador no Rviz
while not rospy.is_shutdown():
        
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0],0,0]
        pub.publish(hello_str)
        rate.sleep()  
