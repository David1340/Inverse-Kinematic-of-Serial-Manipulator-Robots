#PSO aplicado ao pionner 3D
from random import random,uniform
from math import pi,cos,sin,sqrt,atan2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

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

def plot_junta_revolucao(A,p,c):
    #A matriz de Rotação, p origem da junta no seu sistema de coordenadas
    r = 0.025
    h = 0.05
    theta = np.arange(0,2*pi + 0.12,0.8) #theta de 0 a 2pi com passos de 0.1
    if(c == 'z'):  
        z = np.linspace(0,h,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        z, theta = np.meshgrid(z, theta) #transforma em vetores 2D por causa do plot de superficie
        #[x,y,z].T = A@([x,y,z].T + p)
        x = (r*np.cos(theta) + p[0,0])*A[0,0] + (r*np.sin(theta) + p[1,0])*A[0,1] + \
         (z + p[2,0])*A[0,2] + np.ones_like(z)*A[0,3]
        y = (r*np.cos(theta) + p[0,0])*A[1,0] + (r*np.sin(theta) + p[1,0])*A[1,1] + \
         (z + p[2,0])*A[1,2] + np.ones_like(z)*A[1,3]
        z = (r*np.cos(theta) + p[0,0])*A[2,0] + (r*np.sin(theta) + p[1,0])*A[2,1] + \
         (z + p[2,0])*A[2,2] + np.ones_like(z)*A[2,3]
        ax.plot_surface(x, y, z,color = 'blue', alpha = 1)
        
    elif(c == 'y'):
        y = np.linspace(-h,h,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        y, theta = np.meshgrid(y, theta) #transforma em vetores 2D por causa do plot de superficie
        #[x,y,z].T = A@([x,y,z].T + p)
        x = (r*np.cos(theta) + p[0,0])*A[0,0] + (r*np.sin(theta) + p[2,0])*A[0,2] + \
         (y + p[1,0])*A[0,1] + np.ones_like(y)*A[0,3]
        z = (r*np.cos(theta) + p[0,0])*A[2,0] + (r*np.sin(theta) + p[2,0])*A[2,2] + \
         (y + p[1,0])*A[2,1] + np.ones_like(y)*A[2,3]
        y = (r*np.cos(theta) + p[0,0])*A[1,0] + (r*np.sin(theta) + p[2,0])*A[1,2] + \
         (y + p[1,0])*A[1,1] + np.ones_like(y)*A[1,3]
        ax.plot_surface(x, y, z,color = 'blue', alpha = 1)
        
    elif(c == 'x'):
        x = np.linspace(-h,h,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        x, theta = np.meshgrid(x, theta) #transforma em vetores 2D por causa do plot de superficie
        #[x,y,z].T = A@([x,y,z].T + p)
        y = (r*np.cos(theta) + p[2,0])*A[1,2] + (r*np.sin(theta) + p[1,0])*A[1,1] + \
         (x + p[0,0])*A[1,0] + np.ones_like(x)*A[1,3]
        z = (r*np.cos(theta) + p[2,0])*A[2,2] + (r*np.sin(theta) + p[1,0])*A[2,1] + \
         (x + p[0,0])*A[2,0] + np.ones_like(x)*A[2,3]
        x = (r*np.cos(theta) + p[2,0])*A[0,2] + (r*np.sin(theta) + p[1,0])*A[0,1] + \
         (x + p[0,0])*A[0,0] + np.ones_like(x)*A[0,3]
        ax.plot_surface(x, y, z,color = 'blue', alpha = 1)
        
def plot(qbest,t):
    #Comprimento dos elos do manipulador
    b1 = 0.2 #20 cm
    b2 = 0.1
    b3 = 0.2 
    b4 = 0.1
    b5 = 0.2
    b6 = 0.1
    b7 = 0.2
    
    w = 0.05 #largura do efetuador
    h = 0.05 #altura do efetuador
    L = 0.2 #comprimento do elo que liga a junta 7 ao efetuador

    #Pontos de interesse
    p = np.array([[0,0,0,1]]).T #Base
    p1_1 = np.array([[0,-b2,0,1]]).T #junta1
    p2_2 = p #junta2
    p3_3 = np.array([[0,-b4,0,1]]).T #junta3
    p4_4 = p #junta4
    p5_5 = np.array([[0,-b6,0,1]]).T #junta5
    p6_6 = np.array([[-b7,0,0,1]]).T #junta6
    p7_7 = p #junta7
    p8_7 = np.array([[0,0,L,1]]).T #contato de atuação do efetuador
    
    #O efetuador será um u, para plotarmos um precisamos de 4 pontos
    #topo esquerdo do u
    e1_7 = np.array([[0,-w/2,L + h,1]]).T
    #base esquerda do u
    e2_7 = np.array([[0,-w/2,L,1]]).T
    #base direita do u
    e3_7 = np.array([[0,w/2,L,1]]).T
    #topo direito do u
    e4_7 = np.array([[0,w/2,L+ h,1]]).T    
    ### parametros de DH
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
    theta1 = pi/2 + qbest[0]
    theta2 = qbest[1]
    theta3 = qbest[2]
    theta4 = qbest[3]
    theta5 = qbest[4]
    theta6 = pi/2 + qbest[5]
    theta7 = pi/2 + qbest[6]
    #Calculando as matrizes homogêneas
    A1 = matriz_homogenea(d1,a1,alpha1,theta1)
    A2 = matriz_homogenea(d2,a2,alpha2,theta2)
    A3 = matriz_homogenea(d3,a3,alpha3,theta3)
    A4 = matriz_homogenea(d4,a4,alpha4,theta4)
    A5 = matriz_homogenea(d5,a5,alpha5,theta5)
    A6 = matriz_homogenea(d6,a6,alpha6,theta6)
    A7 = matriz_homogenea(d7,a7,alpha7,theta7)
    #Calculando os pontos de interesse no sistema Global
    T1 = A1
    T2 = T1@A2
    T3 = T2@A3
    T4 = T3@A4
    T5 = T4@A5
    T6 = T5@A6
    T7 = T6@A7
    p1_0 = T1@p1_1
    p2_0 = T2@p2_2
    p3_0 = T3@p3_3
    p4_0 = T4@p4_4
    p5_0 = T5@p5_5
    p6_0 = T6@p6_6
    p7_0 = T7@p7_7
    p8_0 = T7@p8_7
    e1_0 = T7@e1_7
    e2_0 = T7@e2_7
    e3_0 = T7@e3_7
    e4_0 = T7@e4_7
    
    #Plotando Elos 
    ax.clear()   
    #Plotando Elos    
    plt.plot([p[0,0],p1_0[0,0]],[p[1,0],p1_0[1,0]],[p[2,0],p1_0[2,0]],'blue')
    plt.plot([p1_0[0,0],p2_0[0,0]],[p1_0[1,0],p2_0[1,0]],[p1_0[2,0],p2_0[2,0]],'blue')
    plt.plot([p2_0[0,0],p3_0[0,0]],[p2_0[1,0],p3_0[1,0]],[p2_0[2,0],p3_0[2,0]],'blue')
    plt.plot([p3_0[0,0],p4_0[0,0]],[p3_0[1,0],p4_0[1,0]],[p3_0[2,0],p4_0[2,0]],'blue')
    plt.plot([p4_0[0,0],p5_0[0,0]],[p4_0[1,0],p5_0[1,0]],[p4_0[2,0],p5_0[2,0]],'blue')
    plt.plot([p5_0[0,0],p6_0[0,0]],[p5_0[1,0],p6_0[1,0]],[p5_0[2,0],p6_0[2,0]],'blue')
    plt.plot([p6_0[0,0],p7_0[0,0]],[p6_0[1,0],p7_0[1,0]],[p6_0[2,0],p7_0[2,0]],'blue')
    
    #Plotando efetuador
    plt.plot([e1_0[0,0],e2_0[0,0],e3_0[0,0],e4_0[0,0],e3_0[0,0],p8_0[0,0],p7_0[0,0]]\
             ,[e1_0[1,0],e2_0[1,0],e3_0[1,0],e4_0[1,0],e3_0[1,0],p8_0[1,0],p7_0[1,0]]\
             ,[e1_0[2,0],e2_0[2,0],e3_0[2,0],e4_0[2,0],e3_0[2,0],p8_0[2,0],p7_0[2,0]],'blue')
    
    #Plotando objetivo    
    ax.scatter(t[0],t[1],t[2],'red')
    
    #Plotando Juntas e base    
    plot_junta_revolucao(np.eye(4),p,'z')
    plot_junta_revolucao(T1,p1_1,'y')
    plot_junta_revolucao(T2,p2_2,'y')
    plot_junta_revolucao(T3,p3_3,'y')
    plot_junta_revolucao(T4,p4_4,'y')
    plot_junta_revolucao(T5,p5_5,'y')
    plot_junta_revolucao(T6,p6_6,'y')
    plot_junta_revolucao(T7,p7_7,'y')

    ax.set_xlabel('Eixo x(m)')
    ax.set_ylabel('Eixo y(m)')
    ax.set_zlabel('Eixo z(m)')

    #titulo
    plt.title('Pioneer 7DOF')
    ax.set_xlim3d(-0.8,0.8)
    ax.set_ylim3d(-0.8,0.8)
    ax.set_zlim3d(0,0.8)
    fig.canvas.draw() #mostra o plot
    plt.pause(0.05)
    
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
        plot(qbest,o)
        #plot(q[0].p,o)
        if(f <= 0.001):
            print("Solução: ",qbest,"em ",j + 1, "interações.\n\n")
            print(f)
            break;        
    
    print(f)
    return qbest

#Main
#configurando o plot
plt.ion()
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')
plt.pause(0.1)

#objetivo
objetivo = np.array([0.2,0.5,0.6]) #posição
objetivo2 = np.array([0,0,0]) #orientação
numero_particulas = 200
dimensao = 7 #dimensão do robô

#restrições de cada ângulo
c = pi/12 
L = [(pi)-c,pi/2,(pi)-c,(pi)-c,(pi)-c,(pi)-c,(pi)-c]
solucao = PSO(objetivo,objetivo2,numero_particulas,dimensao,L)
print('Solução q = ', solucao)

