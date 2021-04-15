# %%
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin, sqrt, pi, atan2
import numpy as np
import math
import random 

#Retorna a Matriz de transformacao Homogeneadados  usando como entrada os parametros de DH
def matriz_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

def matriz_antissimetrica(a):
    #A = [0,-az,ay ; az,0,-ax ; -ay,ax,0]
    A = np.zeros((3,3))
    A[0,1] = -a[2,0]
    A[0,2] = a[1,0]
    A[1,2] = -a[0,0]
    A[1,0] = - A[0,1]
    A[2,0] = - A[0,2]
    A[2,1] = - A[1,2]
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

#Função que plota o manipulador, quando recebe os pontos de interesse
def plot(q,t):
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
    ax.scatter(t[0,0],t[1,0],t[2,0],'red')
    
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


#configurando o plot
plt.ion()
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')

#variaveis
cont = 0
qmax = 0.1
c = 0.5 #tamanho do passo
destino = np.array([[0.2,0.5,0.6]]).T

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

for v in range(1000):
    
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
    
    #os vetores z serao transformados em vetores  no R^3
    z0_0 = k[0:3]
    z1_0 = (T1@k)[0:3]
    z2_0 = (T2@k)[0:3]
    z3_0 = (T3@k)[0:3]
    z4_0 = (T4@k)[0:3]
    z5_0 = (T5@k)[0:3]
    z6_0 = (T6@k)[0:3]
    #z7_0 = (T7@k)[0:3] nao eh usado

    #cálculo do Jacobiano analitico
    J = np.zeros([6,7])
    #produto vetorial de Z0_0 por (o7_0 - o) 
    J[0:3,0] = matriz_antissimetrica(z0_0)@(o7_0[0:3] - o[0:3])[:,0]
    J[3:6,0] = z0_0[:,0]
    J[0:3,1] = matriz_antissimetrica(z1_0)@(o7_0[0:3] - o1_0[0:3])[:,0]
    J[3:6,1] = z1_0[:,0]
    J[0:3,2] = matriz_antissimetrica(z2_0)@(o7_0[0:3] - o2_0[0:3])[:,0]
    J[3:6,2] = z2_0[:,0]
    J[0:3,3] = matriz_antissimetrica(z3_0)@(o7_0[0:3] - o3_0[0:3])[:,0]
    J[3:6,3] = z3_0[:,0]
    J[0:3,4] = matriz_antissimetrica(z4_0)@(o7_0[0:3] - o4_0[0:3])[:,0]
    J[3:6,4] = z4_0[:,0]
    J[0:3,5] = matriz_antissimetrica(z5_0)@(o7_0[0:3] - o5_0[0:3])[:,0]
    J[3:6,5] = z5_0[:,0]
    J[0:3,6] = matriz_antissimetrica(z6_0)@(o7_0[0:3] - o6_0[0:3])[:,0]
    J[3:6,6] = z6_0[:,0]

    f = p_0[0:3] - destino
    J = J[0:3,:] #foi descartado a parte da velocidade angular, porque ela nao esta sendo usada
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
    ax.clear()
    plot(q,destino)
    if(erro < 0.001):
        print('Solucao q: \n',q,'\nNumero de iteracoes:',v)
        break
    #print(erro)
    plt.pause(0.001)
    




    

# %%
