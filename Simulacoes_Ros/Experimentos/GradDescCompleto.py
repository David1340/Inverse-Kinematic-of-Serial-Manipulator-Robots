#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Método do Gradiente Descendente/Transposto 
#para encontrar encontrar uma configuração q
#dada uma posição (x,y,z) com orientação definida
#no espaço para o Pioneer 7DOF
#aplicado ao problema de IK completo (posição e orientação)

#Import do modulo funcoes.py
import sys
sys.path.append('/home/david/Pibic2021/Inverse-Kinematic-of-Serial-Manipulator-Robots/Simulacoes_Ros')
from funcoes import random_pose, matriz_homogenea, distancia, S, orientacao

#Import das bibliotecas python
from math import pi
import numpy as np
import random 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

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
convergencias = 0
media_iteracao = 0
#valor maximo que a junta pode assumir
qlim = [2.6179,1.5358,2.6179,1.6144,2.6179,1.8413,1.7889] 
for cont2 in range (1000):
    convergiu = 0
    #Objetivos
    [posicaod,orientd] = random_pose()
    rpyd = orientacao(orientd)
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
        #hello_str.header.stamp = rospy.Time.now()
        #hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0],0,0]
        #pub.publish(hello_str)
        #rate.sleep()        

        #Condição de parada   
        errop =  distancia(p_0,posicaod,3) #erro de posiçao
        rpy = orientacao(T7[0:3,0:3]) #angulos Roll, Pitch Yall
        erroa = distancia(rpy,rpyd,3) #erro angular
        c1 = 1
        c2 = 0.1
        erro = c1*errop + c2*erroa #composição de erro
        erro =  distancia(p_0,posicaod,3)   
        
        if(erro < 0.001):
            convergiu = 1
            media_iteracao = media_iteracao + cont
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
        f[0:3] = posicaod - p_0[0:3]
        #a parte angular peguei do artigo A closed-loop inverse kinematic scheme
        #online joint based robot control
        f[3:6] = 0.5*(S(T7[0:3,0:1])@orientd[:,0:1]+ S(T7[0:3,1:2])@orientd[:,1:2]  + \
            S(T7[0:3,2:3])@orientd[:,2:3])
        
        #cálculo da constante de passo c
        e = np.array([f[0,0],f[1,0],f[2,0],f[3,0],f[4,0],f[5,0]])
        c = e.dot(J@J.T@e)/((J@J.T@e).dot(J@J.T@e))#peguei essa equacao do
        #artigo Inverse Kinematics Techniques in Computer Graphics: A Survey

        #Equação do Jacobiano Transposto
        dq = c*(J.T@f)

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
    convergencias = convergencias + convergiu  
    print(cont2)    

media_iteracao = media_iteracao/convergencias
print('convergencias: ',convergencias/10)
print('media de iterações: ',media_iteracao)

