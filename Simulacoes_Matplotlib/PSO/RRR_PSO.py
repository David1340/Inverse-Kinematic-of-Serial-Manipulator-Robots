from random import random,uniform
from math import pi,cos,sin,sqrt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

def matriz_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2
        
    return sqrt(d)

def plotagem(qbest,t):
    #vetores colunas do sistema de coordenadas global
    #Pontos de interesse
    o = np.array([[0,0,0,1]]).T 
    o1_1 = o
    o2_2 = o
    o3_3 = o
    ##Parametros dos robos
    b1 = 2
    b2 = 2
    b3 = 2
    ### parametros de DH
    d1 = b1 
    d2 = 0
    d3 = 0
    a1 = 0
    a2 = b2
    a3 = b3
    alfa1 = pi/2
    alfa2 = 0
    alfa3 = 0
    theta1 = pi/2 + qbest[0] 
    theta2 = qbest[1] 
    theta3 = qbest[2]
    #Calculando as matrizes homogêneas
    A1 = matriz_homogenea(d1,a1,alfa1,theta1)
    A2 = matriz_homogenea(d2,a2,alfa2,theta2)
    A3 = matriz_homogenea(d3,a3,alfa3,theta3)
    #Calculando os pontos de interesse no sistema Global
    o1_0 = A1@o1_1
    o2_0 = A1@(A2@o2_2)
    o3_0 = A1@(A2@(A3@o3_3))
    ax.clear()
    #Plotando Elos
    plt.plot([o[0,0],o1_0[0,0]],[o[1,0],o1_0[1,0]],[o[2,0],o1_0[2,0]])
    plt.plot([o1_0[0,0],o2_0[0,0]],[o1_0[1,0],o2_0[1,0]],[o1_0[2,0],o2_0[2,0]])
    plt.plot([o2_0[0,0],o3_0[0,0]],[o2_0[1,0],o3_0[1,0]],[o2_0[2,0],o3_0[2,0]])

    #Plotando Juntas
    ax.scatter(o[0,0],o[1,0],o[2,0])
    ax.scatter(o1_0[0,0],o1_0[1,0],o1_0[2,0])
    ax.scatter(o2_0[0,0],o2_0[1,0],o2_0[2,0])

    #Plotando efetuador
    ax.scatter(o3_0[0,0],o3_0[1,0],o3_0[2,0])
    ax.scatter(t[0],t[1],t[2])
    ax.scatter
    #Legendas
    plt.legend(['Elo 1','Elo 2','Elo 3','Junta 1 (Revolucao)','Junta 2 (Revolucao)'\
                ,'Junta 3 (Revolucao)','Efetuador','Objetivo'])

    ax.set_xlabel('Eixo x')
    ax.set_ylabel('Eixo y')
    ax.set_zlabel('Eixo z')

    #titulo
    plt.title('Manipulador RRR')
    ax.set_xlim3d(-4,4)
    ax.set_ylim3d(-4,4)
    ax.set_zlim3d(0,4)
    fig.canvas.draw() #mostra o plot
    plt.pause(0.01)
    
class particle:
    def __init__(self,position,dimension):
        self.p = position #current position
        self.v = np.zeros(dimension) #current velocity
        self.bp = position.copy() #best position
        self.n = dimension
        self.d = 500 #currente distance
        self.bd = 500 #best distance

    def update_position(self,qbest): #Update the particle postion
        c1 = 1.4047
        c2 = 1.494       
        for i in range(self.n):
            self.v[i] = random()*self.v[i] + c1*random()*(qbest[i] - self.p[i])+ c2*random()*(self.bp[i] - self.p[i])
            self.p[i] = self.p[i] + self.v[i]
            if(self.p[i] > 2*pi):
                self.p[i] = 2*pi
            elif(self.p[i] < 0):
                self.p[i] = 0                
            
    def update_fuction(self,o): #Calcule the value of the fitness fuction
        #Parâmetros físicos do robô
        b1 = 2
        b2 = 2
        b3 = 2
        #posição do efetuador em relação ao ultimo sistema de coordenada
        p = np.array([0,0,0,1]).T
        #Parâmetros de Denavit-Hartenberg do manipulador RRR constantes
        d1 = b1
        d2 = 0
        d3 = 0
        a1 = 0
        a2 = b2
        a3 = b3
        alpha1 = pi/2
        alpha2 = 0
        alpha3 = 0
        theta1 = pi/2 + self.p[0]
        theta2 = self.p[1]
        theta3 = self.p[2]
        A1 = matriz_homogenea(d1,a1,alpha1,theta1)
        A2 = matriz_homogenea(d2,a2,alpha2,theta2)
        A3 = matriz_homogenea(d3,a3,alpha3,theta3)
        #posição do efetuador em relação ao sistema de coordenadas global
        p = A1@(A2@(A3@p))
        #calculo da distancia euclidiana da posição do efetuador em relação ao objetivo
        self.d = distancia(p.T,o,self.n)
        if(self.d < self.bd):
            self.bd = self.d
            self.bp = self.p.copy()

def PSO(o,number,n):
    #limite do valor que a junta pode assumir
    L = 2*pi
    #numero limite de interações
    k = 50       
    q = []
    #criando number particulas de dimensão n e calculando o valor de sua função de custo
    for i in range(number):
        p = np.array([uniform(0,L),uniform(0,L),uniform(0,L)])
        q.append(particle(p,n))
        q[i].update_fuction(o)

    #Criando a configuração qbest e sua distancia
    qbest = q[0].p.copy()
    d = q[0].d
    for i in range(number):
        if(d > q[i].d):
            qbest = q[i].p.copy()
            d = q[i].d
    #Executando PSO
    for j in range(k):

        for i in range(number):          
            q[i].update_position(qbest)
            q[i].update_fuction(o)
            
            if(d > q[i].d):
                qbest = q[i].p.copy()
                d = q[i].d
        plotagem(qbest,o)       
        if(d < 0.01):
            print("Solução: ",qbest,"em ",j + 1, "interações.")
            break;        
        #print("q best = ",qbest, " j = ", j,"\n d = ",d,"\n\n")
    return qbest

#Main
#configurando o plot
plt.ion()
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')
plt.pause(0.1)
#objetivo
objetivo = np.array([-3,1,3.5])
numero_particulas = 20
dimensao = 3
solucao = PSO(objetivo,numero_particulas,dimensao)
