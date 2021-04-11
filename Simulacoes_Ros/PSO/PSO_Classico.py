#PSO aplicado ao pionner 3D
from random import random,uniform
from math import pi,cos,sin,sqrt,atan2
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def matriz_homogenea(d,a,alfa,theta):
    #retorna a matriz homogênea a partir do parâmetros de DH
    c = 5 #numero de casas decimais
    L1 = np.array([round(cos(theta),c), round(-sin(theta)*cos(alfa),c),\
        round(sin(theta)*sin(alfa),c),round(a*cos(theta),c)])
    L2 = np.array([round(sin(theta),c), round(cos(theta)*cos(alfa),c),\
        round(-cos(theta)*sin(alfa),c),round(a*sin(theta),c)])
    L3 = np.array([0, round(sin(alfa),c), round(cos(alfa),c), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2      
    return sqrt(d)

def orientação(A):
     #calcular os ângulos de orientação na conversão Z -> Y -> X
    R = atan2(A[1,0],A[0,0])
    P = atan2(-A[2,0],sqrt((A[2,1]**2)+(A[2,2]**2)))
    Y = atan2(A[2,1],A[2,2])
    result = np.array([R,P,Y])
    return result
            
class particle:
    def __init__(self,position,dimension):
        self.p = position #current position
        self.v = np.zeros(dimension) #current velocity
        self.bp = position.copy() #best position
        self.n = dimension
        self.d = 0 #currente distance
        self.o = np.array([0,0,0])
        self.f = 500 #current fitness fuction
        self.bf = self.f #best fitness fuction

    def update_position(self,qbest,L): #Update the particle position
        #c1 = 1.4047
        #c2 = 1.494
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
       
            
    def update_fuction(self,o,o2): #Calcule the value of the fitness fuction
        #Parâmetros físicos do robô, comprimento dos links
        b1 = 0.2 #20 cm
        b2 = 0.1
        b3 = 0.2 
        b4 = 0.1
        b5 = 0.2
        b6 = 0.1
        b7 = 0.2
        L = 0.2 

        #centro do efetuador em relação ao ultimo sistema de coordenada
        p = np.array([0,0,L,1]).T

        #Parâmetros de Denavit-Hartenberg do manipulado
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

        #orientação
        self.o = distancia(orientação(A),o2,3)

        #calculo da distancia euclidiana da posição do efetuador em relação ao objetivo
        self.d = distancia(p.T,o,3)

        #função de custo       
        k1 = 0.2 #orientação
        k2 = 0.8 #posição
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

    #Criando a configuração qbest e sua distancia
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
            
            if(f > q[i].f):
                #print("d: ",q[i].d,"\n")
                #print("o: ",q[i].o,"\n")
                #print("qbest: ",q[i].p,"\n\n")
                qbest = q[i].p.copy()
                f = q[i].f
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [qbest[0],qbest[1],qbest[2],qbest[3],qbest[4],qbest[5],qbest[6],0,0]
        pub.publish(hello_str)
        rate.sleep()
        if(f <= 0.001):
            print("Solução: ",qbest,"em ",j + 1, "interações.\n\n")
            print(f)
            break;        
    
    print(f)
    return qbest

#Main
#configurando o Rviz
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(10) # 10hz
hello_str = JointState()
hello_str.header = Header()
hello_str.header.stamp = rospy.Time.now()
hello_str.name = ['One_joint', 'Two_joint', 'Three_joint', 'Four_joint'\
    ,'Five_joint','Six_joint','Seven_joint','L_F_joint','R_F_joint']
hello_str.position  = [0,0,0,0,0,0,0,0,0]    
hello_str.velocity = []
hello_str.effort = []

#objetivo
objetivo = np.array([0.2,0.5,0.6]) #posição
objetivo2 = np.array([0,0,0]) #orientação
numero_particulas = 200
dimensao = 7 #dimensão do robô

#restrições de cada ângulo
c = pi/12 
L = [(pi)-c,pi/2,(pi)-c,(pi)-c,(pi)-c,(pi)-c,(pi)-c]
while not rospy.is_shutdown():
    solucao = PSO(objetivo,objetivo2,numero_particulas,dimensao,L)
    print('Solução q = ', solucao)
    break
