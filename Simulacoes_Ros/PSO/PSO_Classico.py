#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Particle Swarm Optimizarion
#para encontrar encontrar uma configuração q
#dada uma posição (x,y,z) e uma orientacao 
#no espaço para o Pioneer 7DOF

#Import do modulo funcoes.py
import sys
sys.path.append('/home/david/Pibic2021/Inverse-Kinematic-of-Serial-Manipulator-Robots/Simulacoes_Ros')
from funcoes import random_pose, matriz_homogenea, distancia, orientacao

#Import das bibliotecas python
from random import random,uniform
from math import pi
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class particle:
    def __init__(self,position,dimension):
        self.p = position #posição atual da particula/configuração do robô
        self.v = np.zeros(dimension) #velocidade atual da particula
        self.bp = position.copy() #melhor posição que a particula ja esteve
        self.n = dimension #dimensão da particula
        self.d = 0 #Diferença em módulo da distância atual para a desejada
        self.o = np.array([0,0,0]) #Diferença em módulo da orientacao atual para a desejada
        self.f = 500 #Função de custo/fitnees atual da particula
        self.bf = self.f #Melhor valor de função de custo da obtida pela particula

    def update_position(self,qbest,L): #Atualiza a posição da particula/configuração do robô
        c1 = 1 #grupo
        c2 = 2 #individual
        for i in range(self.n):
            w = 0.5 + random()/2
            vmax = 0.5
            #w = random()
            self.v[i] = w*self.v[i] + c1*random()*(qbest[i] - self.p[i])+ c2*random()*(self.bp[i] - self.p[i])
            if(self.v[i] > vmax):
                self.v[i] = vmax
            elif(self.v[i] < -vmax):
                self.v[i] = -vmax
            self.p[i] = self.p[i] + self.v[i]
            if(self.p[i] > L[i]):
                self.p[i] = L[i]
            elif(self.p[i] < -L[i]):
                self.p[i] = -L[i]
       
            
    def update_fuction(self,o,o2): #Calcula a função de custo/fitness da particula
        #(posição,orientacao) da pose desejada
        #Parâmetros Físicos do manipulador [m]
        base = 0.05 #5 cm
        L = 0.075 #distância da ultima junta a extremidade do efetuador

        #centro do efetuador em relação ao ultimo sistema de coordenada
        p = np.array([0,0,L,1]).T

        #Parâmetros de Denavit-Hartenberg do manipulado
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
        theta1 = pi/2 + self.p[0]
        theta2 = self.p[1]
        theta3 = self.p[2]
        theta4 = self.p[3]
        theta5 = self.p[4]
        theta6 = pi/2 + self.p[5]
        theta7 = pi/2 + self.p[6]

        #Calculo das matrizes homogeneas
        A1 = matriz_homogenea(d1,a1,alpha1,theta1)
        A2 = matriz_homogenea(d2,a2,alpha2,theta2)
        A3 = matriz_homogenea(d3,a3,alpha3,theta3)
        A4 = matriz_homogenea(d4,a4,alpha4,theta4)
        A5 = matriz_homogenea(d5,a5,alpha5,theta5)
        A6 = matriz_homogenea(d6,a6,alpha6,theta6)
        A7 = matriz_homogenea(d7,a7,alpha7,theta7)
        A = A1@(A2@(A3@(A4@(A5@(A6@A7)))))

        #posição do efetuador em relação ao sistema de coordenadas global
        p = A@p

        #calculo do erro em módulo da orientacao desejada e da particula
        self.o = distancia(orientacao(A),o2,3)

        #calculo da distancia euclidiana da posição do efetuador em relação ao objetivo
        self.d = distancia(p.T,o,3)

        #Calculo da função de custo       
        k1 = 0.1 #orientacao
        k2 = 0.9 #posição
        self.f = (k1*self.o) + (k2*self.d)
        if(self.f < self.bf):
            self.bf = self.f
            self.bp = self.p.copy()

def PSO(o,o2,number,n,L):
    #numero limite de interações
    k = 250      
    q = []
    
    #criando as particulas de dimensão n e calculando o valor de sua função de custo
    for i in range(number):
        p = np.array([uniform(-L[0],L[0]),uniform(-L[1],L[1]),uniform(-L[2],L[2]),uniform(-L[3],L[3])\
                      ,uniform(-L[4],L[4]),uniform(-L[5],L[5]),uniform(-L[6],L[6])])
        q.append(particle(p,n))
        q[i].update_fuction(o,o2)

    #Criando a configuração qbest e sua função de custo 
    qbest = q[0].p.copy()
    f = q[0].f
    for i in range(number):
        if(f > q[i].f):
            qbest = q[i].p.copy()
            f = q[i].f
            
    #Executando PSO
    for j in range(k):
        for i in range(number):          
            q[i].update_position(qbest,L)
            q[i].update_fuction(o,o2)
            
            #Se alguma particula possui função de custo menor do que qbest, ela se torna a qbest
            if(f > q[i].f): 
                qbest = q[i].p.copy()
                f = q[i].f
        #Atualiza a configuração do robô no Rviz
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [qbest[0],qbest[1],qbest[2],qbest[3],qbest[4],qbest[5],qbest[6],0,0]
        pub.publish(hello_str)
        rate.sleep()
        #Critério de parada
        if(f <= 0.001):
            print("Solução: ",qbest,"em ",j + 1, "interações.\n\n")
            break;            
    
    print(f)
    return qbest

#Main
#configurando o Rviz
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(100) # 10hz
hello_str = JointState()
hello_str.header = Header()
hello_str.header.stamp = rospy.Time.now()
hello_str.name = ['One_joint', 'Two_joint', 'Three_joint', 'Four_joint'\
    ,'Five_joint','Six_joint','Seven_joint','L_F_joint','R_F_joint']
hello_str.position  = [0,0,0,0,0,0,0,0,0]    
hello_str.velocity = []
hello_str.effort = []

#objetivo
[posicaod,orientd] = random_pose()
orientd = orientacao(orientd)
numero_particulas = 200
dimensao = 7 #dimensão do robô

#restrições de cada ângulo
L = [2.6179,1.5358,2.6179,1.6144,2.6179,1.8413,1.7889]
while not rospy.is_shutdown():
    solucao = PSO(posicaod,orientd,numero_particulas,dimensao,L)
    print('Solução q = ', solucao)
    break


